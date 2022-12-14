/*
    LV2 Parameter code from:
    https://github.com/lv2/lv2/tree/master/plugins/eg-params.lv2

    LV2 Parameter Example Plugin
    Copyright 2014-2016 David Robillard <d@drobilla.net>

    Permission to use, copy, modify, and/or distribute this software for any
    purpose with or without fee is hereby granted, provided that the above
    copyright notice and this permission notice appear in all copies.

    THIS SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
    WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
    MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
    ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
    WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
    ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
    OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdatomic.h>

#include "state_map.h"

#include "lv2/atom/atom.h"
#include "lv2/atom/forge.h"
#include "lv2/atom/util.h"
#include "lv2/core/lv2.h"
#include "lv2/core/lv2_util.h"
#include "lv2/log/log.h"
#include "lv2/log/logger.h"
#include "lv2/midi/midi.h"
#include "lv2/patch/patch.h"
#include "lv2/state/state.h"
#include "lv2/urid/urid.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lv2-hmi.h"

#define PLUGIN_URI "http://moddevices.com/plugins/mod-devel/mod-advanced-control-to-cv"

#define N_PROPS             1
#define MAX_STRING          1024

#define UNIT_STRING_URI         PLUGIN_URI "#unitstring"

#define SPECIAL_PORT_RESET      UINT8_MAX

#define UNIT_STRING_TEXT        "VOLT"

typedef struct {
    LV2_URID plugin;
    LV2_URID atom_Path;
    LV2_URID atom_Sequence;
    LV2_URID atom_URID;
    LV2_URID atom_eventTransfer;
    LV2_URID atom_String;
    LV2_URID atom_Int;
    LV2_URID midi_Event;
    LV2_URID patch_Get;
    LV2_URID patch_Set;
    LV2_URID patch_Put;
    LV2_URID patch_body;
    LV2_URID patch_subject;
    LV2_URID patch_property;
    LV2_URID patch_value;
    LV2_URID state_StateChanged;
    LV2_URID unit_string;
} URIs;

typedef struct {
    LV2_Atom        unitstring;
    char            unitstring_data[MAX_STRING];
} State;

typedef struct {
    uint32_t size;
    void *body;
} restore_value;

static inline void
map_uris(LV2_URID_Map* map, URIs* uris)
{
    uris->plugin = map->map(map->handle, PLUGIN_URI);

    uris->atom_Path          = map->map(map->handle, LV2_ATOM__Path);
    uris->atom_Sequence      = map->map(map->handle, LV2_ATOM__Sequence);
    uris->atom_URID          = map->map(map->handle, LV2_ATOM__URID);
    uris->atom_eventTransfer = map->map(map->handle, LV2_ATOM__eventTransfer);
    uris->atom_String        = map->map(map->handle, LV2_ATOM__String);
    uris->atom_Int           = map->map(map->handle, LV2_ATOM__Int);
    uris->midi_Event         = map->map(map->handle, LV2_MIDI__MidiEvent);
    uris->patch_Get          = map->map(map->handle, LV2_PATCH__Get);
    uris->patch_Set          = map->map(map->handle, LV2_PATCH__Set);
    uris->patch_Put          = map->map(map->handle, LV2_PATCH__Put);
    uris->patch_body         = map->map(map->handle, LV2_PATCH__body);
    uris->patch_subject      = map->map(map->handle, LV2_PATCH__subject);
    uris->patch_property     = map->map(map->handle, LV2_PATCH__property);
    uris->patch_value        = map->map(map->handle, LV2_PATCH__value);
    uris->state_StateChanged = map->map(map->handle, LV2_STATE__StateChanged);

    uris->unit_string       = map->map(map->handle, UNIT_STRING_URI);
}

typedef enum {
    Cvoutput = 0,
    Knob,
    Smoothing,
    Min,
    Max,
    PARAMS_IN,
    PARAMS_OUT,
    ROUND
} PortIndex;

typedef struct {
    
    //main knob
    const float* level;

    // CV signals
    float* output;

    //controls
    const float* min;
    const float* max;
    const float* smooth;
    const float *round;

    double a0;
    double b1;
    double z1;

    bool state_changed;

    float prev_value;
    float prev_min;
    float prev_max;
    int prev_round;

    // Features
    LV2_URID_Map*  map;
    LV2_Log_Logger logger;
    LV2_HMI_WidgetControl* hmi;
    
    LV2_Atom_Forge forge;
    LV2_Atom_Forge_Ref ref;

    // Ports
    const LV2_Atom_Sequence* in_port;
    LV2_Atom_Sequence*       out_port;
    LV2_Atom_Forge_Frame notify_frame;

    // URIs
    URIs uris;

    // Plugin state
    StateMapItem props[N_PROPS];
    State        state;

    // HMI Widgets stuff
    LV2_HMI_Addressing control_addressing;
} Control;

static double
lowPassProcess(Control* self, float input)
{
    return self->z1 = input * self->a0 + self->z1 * self->b1;
}

float MAP(float x, float Imin, float Imax, float Omin, float Omax)
{
    return (( x - Imin ) * (Omax -  Omin)  / (Imax - Imin) + Omin);
}

//MOD products only support ascii 32 to 126
void check_string(char *text)
{
    int char_lenght = strlen(text);
    int ascii = 0;
    for (int i = 0; i < char_lenght; i++) {
        ascii = (int)text[i];

        //replace chars with -
        if (ascii < 32)
            text[i] = '-';

        //dont do quotation marks as they are tricky
        if (ascii == 34)
            text[i] = '-';

        if (ascii > 126)
            text[i] = '-';
    }
}

static char* reverse(char* str, uint32_t str_len)
{
    char *end = str + (str_len - 1);
    char *start = str, tmp;

    while (start < end)
    {
        tmp = *end;
        *end = *start;
        *start = tmp;

        start++;
        end--;
    }

    return str;
}

uint32_t int_to_str(int32_t num, char *string, uint32_t string_size, uint8_t zero_leading)
{
    char *pstr = string;
    uint8_t signal = 0;
    uint32_t str_len;

    if (!string) return 0;

    // exception case: number is zero
    if (num == 0)
    {
        *pstr++ = '0';
        if (zero_leading) zero_leading--;
    }

    // need minus signal?
    if (num < 0)
    {
        num = -num;
        signal = 1;
        string_size--;
    }

    // composes the string
    while (num)
    {
        *pstr++ = (num % 10) + '0';
        num /= 10;

        if (--string_size == 0) break;
        if (zero_leading) zero_leading--;
    }

    // checks buffer size
    if (string_size == 0)
    {
        *string = 0;
        return 0;
    }

    // fills the zeros leading
    while (zero_leading--) *pstr++ = '0';

    // put the minus if necessary
    if (signal) *pstr++ = '-';
    *pstr = 0;

    // invert the string characters
    str_len = (pstr - string);
    reverse(string, str_len);

    return str_len;
}

uint32_t float_to_str(float num, char *string, uint32_t string_size, uint8_t precision)
{
    double intp, fracp;
    char *str = string;

    if (!string) return 0;

    // TODO: check Nan and Inf

    // splits integer and fractional parts
    fracp = modf(num, &intp);

    // convert to absolute value
    if (intp < 0.0) intp = -intp;
    if (fracp < 0.0) fracp = -fracp;

    // insert minus if negative number
    if (num < 0.0)
    {
        *str = '-';
        str++;
    }

    // convert the integer part to string
    uint32_t int_len;
    int_len = int_to_str((int32_t)intp, str, string_size, 0);

    // checks if convertion fail
    if (int_len == 0)
    {
        *string = 0;
        return 0;
    }

    // adds one to avoid lost the leading zeros
    fracp += 1.0;

    // calculates the precision
    while (precision--)
    {
        fracp *= 10;
    }

    // add 0.5 to round
    fracp += 0.5;

    // convert the fractional part
    uint32_t frac_len;
    frac_len = int_to_str((int32_t)fracp, &str[int_len], string_size - int_len, 0);

    // checks if convertion fail
    if (frac_len == 0)
    {
        *string = 0;
        return 0;
    }

    // inserts the dot covering the extra one added
    str[int_len] = '.';

    return (int_len + frac_len);
}

void update_screen_value(Control* self)
{
    char bfr[8];
    if ((int)*self->round == 1){
        float minRound = roundf(*self->min);
        float maxRound = roundf(*self->max);
        int screen_value = roundf(MAP(*self->level, 0, 10, minRound, maxRound));
        int_to_str(screen_value, bfr, sizeof(bfr), 0);
    }
    else {
        float screen_value = MAP(*self->level, 0, 10, *self->min, *self->max);
        
        //mimic MOD Dwarf HMI behaviour
        if ((screen_value > 99.99) || (screen_value < -99.99))
            float_to_str((screen_value), bfr, sizeof(bfr), 1);
        else if ((screen_value > 9.99) || (screen_value < -9.99))
            float_to_str((screen_value), bfr, sizeof(bfr), 2);
        else
            float_to_str((screen_value), bfr, sizeof(bfr), 3);
    }

    //change HMI
    self->hmi->set_value(self->hmi->handle, self->control_addressing, bfr);

    self->prev_value = *self->level;
    self->prev_min = *self->min;
    self->prev_max = *self->max;
    self->prev_round = *self->round;
}

static LV2_Handle
instantiate(const LV2_Descriptor*     descriptor,
            double                    rate,
            const char*               bundle_path,
            const LV2_Feature* const* features)
{
    Control* self = (Control*)calloc(sizeof(Control), 1);

    // Get host features
    // clang-format off
    const char* missing = lv2_features_query(
            features,
            LV2_LOG__log,           &self->logger.log,  false,
            LV2_URID__map,          &self->map,         true,
            LV2_HMI__WidgetControl, &self->hmi,         false,
            NULL);
    // clang-format on

    lv2_log_logger_set_map(&self->logger, self->map);

    if (missing) {
        lv2_log_error(&self->logger, "Missing feature <%s>\n", missing);
        free(self);
        return NULL;
    }

    // Map URIs and initialise forge
    map_uris(self->map, &self->uris);
    lv2_atom_forge_init(&self->forge, self->map);

    // Initialise state dictionary
    // clang-format off
    State* state = &self->state;
    state_map_init(
        self->props, self->map, self->map->handle,
        UNIT_STRING_URI, STATE_MAP_INIT(String, &state->unitstring),
        NULL);
    // clang-format on

    self->z1 = 0.0;
    double frequency = 550.0 / rate;
    self->b1 = exp(-2.0 * M_PI * frequency);
    self->a0 = 1.0 - self->b1;

    return (LV2_Handle)self;
}

static LV2_State_Status
set_parameter(Control*     self,
              LV2_URID    key,
              uint32_t    size,
              LV2_URID    type,
              const void* body,
              bool        from_state)
{
    // Look up property in state dictionary
    const StateMapItem* entry = state_map_find(self->props, N_PROPS, key);

    // Set property value in state dictionary
    lv2_log_trace(&self->logger, "Set <%s>\n", entry->uri);
    memcpy(entry->value + 1, body, size);
    entry->value->size = size;
    self->state_changed = true;
    return LV2_STATE_SUCCESS;
}

static const LV2_Atom*
get_parameter(Control* self, LV2_URID key)
{
    const StateMapItem* entry = state_map_find(self->props, N_PROPS, key);
    if (entry) {
        lv2_log_trace(&self->logger, "Get <%s>\n", entry->uri);
        return entry->value;
    }

    return NULL;
}

static LV2_State_Status
write_param_to_forge(LV2_State_Handle handle,
                     uint32_t         key,
                     const void*      value,
                     size_t           size,
                     uint32_t         type,
                     uint32_t         flags)
{
    LV2_Atom_Forge* forge = (LV2_Atom_Forge*)handle;

    if (!lv2_atom_forge_key(forge, key) ||
        !lv2_atom_forge_atom(forge, size, type) ||
        !lv2_atom_forge_write(forge, value, size)) {
        return LV2_STATE_ERR_UNKNOWN;
    }

    return LV2_STATE_SUCCESS;
}

static void
store_prop(Control*                 self,
           LV2_State_Map_Path*      map_path,
           LV2_State_Status*        save_status,
           LV2_State_Store_Function store,
           LV2_State_Handle         handle,
           LV2_URID                 key,
           const LV2_Atom*          value)
{
    LV2_State_Status st = LV2_STATE_SUCCESS;
    
    // Store simple property
    st = store(handle,
               key,
               value + 1,
               value->size,
               value->type,
               LV2_STATE_IS_POD | LV2_STATE_IS_PORTABLE);
  

    if (save_status && !*save_status) {
        *save_status = st;
    }
}

/**
   State save method.
   This is used in the usual way when called by the host to save plugin state,
   but also internally for writing messages in the audio thread by passing a
   "store" function which actually writes the description to the forge.
*/
static LV2_State_Status
save(LV2_Handle                instance,
     LV2_State_Store_Function  store,
     LV2_State_Handle          handle,
     uint32_t                  flags,
     const LV2_Feature* const* features)
{
    Control*             self = (Control*)instance;
    LV2_State_Map_Path* map_path =
        (LV2_State_Map_Path*)lv2_features_data(features, LV2_STATE__mapPath);

    LV2_State_Status st = LV2_STATE_SUCCESS;
    for (unsigned i = 0; i < N_PROPS; ++i) {
        StateMapItem* prop = &self->props[i];
        store_prop(self, map_path, &st, store, handle, prop->urid, prop->value);
    }

    return st;
}

static void
retrieve_prop(Control*                     self,
              LV2_State_Status*           restore_status,
              LV2_State_Retrieve_Function retrieve,
              LV2_State_Handle            handle,
              LV2_URID                    key,
              const LV2_Feature* const*   features)
{
    // Retrieve value from saved state
    size_t      vsize  = 0;
    uint32_t    vtype  = 0;
    uint32_t    vflags = 0;
    const void* value  = retrieve(handle, key, &vsize, &vtype, &vflags);

    // Set plugin instance state
    const LV2_State_Status st =
        value ? set_parameter(self, key, vsize, vtype, value, true)
                : LV2_STATE_ERR_NO_PROPERTY;

    if (!*restore_status) {
        *restore_status = st; // Set status if there has been no error yet
    }
}

/** State restore method. */
static LV2_State_Status
restore(LV2_Handle                  instance,
        LV2_State_Retrieve_Function retrieve,
        LV2_State_Handle            handle,
        uint32_t                    flags,
        const LV2_Feature* const*   features)
{
  Control*         self = (Control*)instance;
  LV2_State_Status st   = LV2_STATE_SUCCESS;

  for (unsigned i = 0; i < N_PROPS; ++i) {
    retrieve_prop(self, &st, retrieve, handle, self->props[i].urid, features);
  }

  self->state_changed = true;

  return st;
}

static inline bool
subject_is_plugin(Control* self, const LV2_Atom_URID* subject)
{
    // This simple plugin only supports one subject: itself
    return (!subject || (subject->atom.type == self->uris.atom_URID &&
                         subject->body == self->uris.plugin));
}

static void
connect_port(LV2_Handle instance,
             uint32_t   port,
             void*      data)
{
    Control* self = (Control*)instance;

    switch ((PortIndex)port) {
        case Cvoutput:
            self->output = (float*)data;
            break;
        case Knob:
            self->level = (float*)data;
            break;
        case Smoothing:
            self->smooth = (const float*)data;
            break;
        case Min:
            self->min = (const float*)data;
            break;
        case Max:
            self->max = (const float*)data;
            break;
        case PARAMS_IN:
            self->in_port = (const LV2_Atom_Sequence*)data;
            break;
        case PARAMS_OUT:
            self->out_port = (LV2_Atom_Sequence*)data;
            break;
        case ROUND:
            self->round = (const float*)data;
            break;
    }
}

static void
activate(LV2_Handle instance)
{
}

static void
run(LV2_Handle instance, uint32_t n_samples)
{
    Control* self = (Control*) instance;
    URIs*    uris = &self->uris;
    LV2_Atom_Forge* forge = &self->forge;

    // Initially, self->out_port contains a Chunk with size set to capacity
    // Set up forge to write directly to output port
    const uint32_t out_capacity = self->out_port->atom.size;
    lv2_atom_forge_set_buffer(forge, (uint8_t*)self->out_port, out_capacity);
    lv2_atom_forge_sequence_head(forge, &self->notify_frame, 0);

    // Start a sequence in the output port
    LV2_Atom_Forge_Frame out_frame;
    lv2_atom_forge_sequence_head(forge, &out_frame, 0);

    // Read incoming events
    LV2_ATOM_SEQUENCE_FOREACH (self->in_port, ev) {
        const LV2_Atom_Object* obj = (const LV2_Atom_Object*)&ev->body;
        if (obj->body.otype == uris->patch_Set) {
            // Get the property and value of the set message
            const LV2_Atom_URID* subject  = NULL;
            const LV2_Atom_URID* property = NULL;
            const LV2_Atom*      value    = NULL;

            // clang-format off
            lv2_atom_object_get(obj,
                                uris->patch_subject,  (const LV2_Atom**)&subject,
                                uris->patch_property, (const LV2_Atom**)&property,
                                uris->patch_value,    &value,
                                0);
            // clang-format on

            if (!subject_is_plugin(self, subject)) {
                lv2_log_error(&self->logger, "Set for unknown subject\n");
            }
            else if (!property) {
                lv2_log_error(&self->logger, "Set with no property\n");
            }
            else if (property->atom.type != uris->atom_URID) {
                lv2_log_error(&self->logger, "Set property is not a URID\n");
            }
            else {
                // Set property to the given value
                const LV2_URID key = property->body;
                set_parameter(self, key, value->size, value->type, value + 1, false);
            }
        }
        else if (obj->body.otype == uris->patch_Get) {
            // Get the property of the get message
            const LV2_Atom_URID* subject  = NULL;
            const LV2_Atom_URID* property = NULL;

            // clang-format off
            lv2_atom_object_get(obj,
                                uris->patch_subject,  (const LV2_Atom**)&subject,
                                uris->patch_property, (const LV2_Atom**)&property,
                                0);
            // clang-format on

            if (!subject_is_plugin(self, subject)) {
                lv2_log_error(&self->logger, "Get with unknown subject\n");
            }
            else if (!property) {
                // Get with no property, emit complete state
                lv2_atom_forge_frame_time(forge, ev->time.frames);
                LV2_Atom_Forge_Frame pframe;
                lv2_atom_forge_object(forge, &pframe, 0, uris->patch_Put);
                lv2_atom_forge_key(forge, uris->patch_body);

                LV2_Atom_Forge_Frame bframe;
                lv2_atom_forge_object(forge, &bframe, 0, 0);
                save(self, write_param_to_forge, forge, 0, NULL);

                lv2_atom_forge_pop(forge, &bframe);
                lv2_atom_forge_pop(forge, &pframe);
            }
            else if (property->atom.type != uris->atom_URID) {
                lv2_log_error(&self->logger, "Get property is not a URID\n");
            }
            else {
                // Get for a specific property
                const LV2_URID  key   = property->body;
                const LV2_Atom* value = get_parameter(self, key);
                if (value) {
                    lv2_atom_forge_frame_time(forge, ev->time.frames);
                    LV2_Atom_Forge_Frame frame;
                    lv2_atom_forge_object(forge, &frame, 0, uris->patch_Set);
                    lv2_atom_forge_key(forge, uris->patch_property);
                    lv2_atom_forge_urid(forge, property->body);
                    store_prop(self,
                               NULL,
                               NULL,
                               write_param_to_forge,
                               forge,
                               uris->patch_value,
                               value);
                    lv2_atom_forge_pop(forge, &frame);
                }
            }
        }
    }

    // notify of state change
    if (self->state_changed) {
        const uint32_t last_frame = n_samples-1;

        // string 1
        {
            lv2_atom_forge_frame_time(forge, last_frame);
            LV2_Atom_Forge_Frame frame;
            lv2_atom_forge_object(forge, &frame, 0, uris->patch_Set);
            lv2_atom_forge_key(forge, uris->patch_property);
            lv2_atom_forge_urid(forge, uris->unit_string);
            lv2_atom_forge_key(forge, uris->patch_value);
            lv2_atom_forge_string(forge, self->state.unitstring_data, strlen(self->state.unitstring_data)+1);
            lv2_atom_forge_pop(forge, &frame);
        }

        char *unit = self->state.unitstring_data;
        if (unit[0] == '\0')
                strcpy(unit, UNIT_STRING_TEXT);

        //sanity check for the chars we want to display
        check_string(unit);

        self->hmi->set_unit(self->hmi->handle, self->control_addressing, unit);

        self->state_changed = false;
    }

    float coef = *self->level;

    for ( uint32_t i = 0; i < n_samples; i++)
    {
        float smooth = lowPassProcess(self, coef);

        if ((int)*self->smooth == 1) {
            coef = smooth;
        }

        self->output[i] = coef;
    }

    //update screen value
    if ((*self->level != self->prev_value) ||
        (*self->min != self->prev_min) ||
        (*self->max != self->prev_max) ||
        (*self->round != self->prev_round))
    {
        update_screen_value(self);
    }

    lv2_atom_forge_pop(forge, &out_frame);
}

static void
deactivate(LV2_Handle instance)
{
}

static void
cleanup(LV2_Handle instance)
{
    free(instance);
}

static void
addressed(LV2_Handle handle, uint32_t index, LV2_HMI_Addressing addressing, const LV2_HMI_AddressingInfo* info)
{
    Control* self = (Control*) handle;

    if (index == Knob) {
        self->control_addressing = addressing;

        update_screen_value(self);

        char *unit = self->state.unitstring_data;
        if (unit[0] == '\0')
                strcpy(unit, UNIT_STRING_TEXT);

        //sanity check for the chars we want to display
        check_string(unit);

        self->hmi->set_unit(self->hmi->handle, self->control_addressing, unit);
    }
}

static void
unaddressed(LV2_Handle handle, uint32_t index)
{
    Control* self = (Control*) handle;

    if (index == Knob)
        self->control_addressing = NULL;
}

static const void*
extension_data(const char* uri)
{
    static const LV2_HMI_PluginNotification hmiNotif = {
        addressed,
        unaddressed,
    };
    if (!strcmp(uri, LV2_HMI__PluginNotification))
        return &hmiNotif;

    static const LV2_State_Interface state = {save, restore};
    if (!strcmp(uri, LV2_STATE__interface)) {
        return &state;
    }

    return NULL;
}

static const LV2_Descriptor descriptor = {
    PLUGIN_URI,
    instantiate,
    connect_port,
    activate,
    run,
    deactivate,
    cleanup,
    extension_data
};

LV2_SYMBOL_EXPORT
const LV2_Descriptor*
lv2_descriptor(uint32_t index)
{
    switch (index) {
        case 0:  return &descriptor;
        default: return NULL;
    }
}
