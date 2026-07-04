//*****************************************************************************
// Title        : Pulse to tone (DTMF) converter
// Author       : Boris Cherkasskiy
//                http://boris0.blogspot.ca/2013/09/rotary-dial-for-digital-age.html
// Created      : 2011-10-24
//
// Modified     : Arnie Weber 2015-06-22
//                https://bitbucket.org/310weber/rotary_dial/
//                NOTE: This code is not compatible with Boris's original hardware
//                due to changed pin-out (see Eagle files for details)
//
// Modified     : Matthew Millman 2018-05-29
//                http://tech.mattmillman.com/
//                Cleaned up implementation, modified to work more like the
//                Rotatone commercial product.
//
// Modified     : davidpty 2026-04-23
//                added hotdial, lock dial and support for 'Erdtaste' 
//
// This code is distributed under the GNU Public License
// which can be found at http://www.gnu.org/licenses/gpl.txt
//
// DTMF generator logic is loosely based on the AVR314 app note from Atmel
//
//*****************************************************************************

// Uncomment to build with reverse dial
//#define NZ_DIAL

#include <stdbool.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <string.h>

#include "dtmf.h" 

#define PIN_DIAL                    PB1
#define PIN_PULSE                   PB2

#define SPEED_DIAL_SIZE             42

#define STATE_DIAL                  0x00
#define STATE_SPECIAL_L1            0x01
#define STATE_SPECIAL_L2            0x02
#define STATE_PROGRAM_SD            0x03
#define STATE_HOTLINE_SLOT          0x04
#define STATE_HOTLINE_DELAY         0x05
#define STATE_PROGRAM_ERDTASTE      0x06

#define F_NONE                      0x00
#define F_DETECT_SPECIAL_L1         0x01
#define F_DETECT_SPECIAL_L2         0x02
#define F_WDT_AWAKE                 0x04

#define SLEEP_64MS                  0x00
#define SLEEP_128MS                 0x01
#define SLEEP_1S                    0x02
#define SLEEP_2S                    0x03

#define SPECIAL_L2_HOLD_TIME        SLEEP_2S

#define DTMF_DURATION_MS_SHORT      100
#define DTMF_DURATION_MS_LONG       200
#define DTMF_PAUSE_MS               2000  // Pause inserted in speed dial memory

#define PAUSE_DETECT_HOLD_TIME      SLEEP_2S  // ~2s WDT interval for pause detection
#define F_DETECT_PAUSE              0x08

#define SPEED_DIAL_COUNT            9   // 9 Positions in total (Redail(3),4,5,6,7,8,9,0 and Erdtaste)
#define SPEED_DIAL_ERDTASTE         (SPEED_DIAL_COUNT - 2)   // slot 7
#define SPEED_DIAL_REDIAL           (SPEED_DIAL_COUNT - 1)   // slot 8

#define L2_STAR                     1
#define L2_POUND                    2
#define L2_REDIAL                   3

#define LOCK_NONE                   0
#define LOCK_PROGRAM                1
#define LOCK_FULL                   2

typedef struct
{
    uint8_t enabled;
    uint8_t slot_digit;
    uint8_t delay_digit;
} hotline_config_t;

typedef struct
{
    uint8_t state;
    uint8_t flags;
    bool dial_pin_state;
    bool suppress_first_release;
    bool program_special_insert;
    bool special_l1_latched;
    uint8_t lock_state;
    int8_t speed_dial_digits[SPEED_DIAL_SIZE];
    int8_t dialed_digit;
    int8_t service_code_buf[5];
    uint8_t special_hold_state;
    uint8_t speed_dial_index;
    uint8_t speed_dial_digit_index;
    uint8_t hotline_slot_digit;
    uint8_t l1_hold_time;
    uint8_t pending_pauses;
    uint16_t dtmf_duration;
} runstate_t;

static void init(void);
static void process_dialed_digit(runstate_t *rs);
static void read_speed_dial(uint8_t slot, int8_t *buf);
static void write_speed_dial(uint8_t slot, int8_t *buf);
static uint8_t find_end(int8_t *buf);
static void load_into_current_slot(int8_t *src, runstate_t *rs);
static bool is_programming_state(uint8_t state);
static uint8_t current_program_slot(runstate_t *rs);
static void clear_current_program_buffer(runstate_t *rs);
static void store_digit_to_redial(runstate_t *rs, int8_t digit);
static void push_to_service_buffer(runstate_t *rs, int8_t digit);
static void clear_speed_dial_slot(runstate_t *rs, uint8_t slot_index);
static void append_digit_to_slot(runstate_t *rs, int8_t digit, uint8_t tone);
static void handle_dial_state(runstate_t *rs, int8_t *digit_to_buffer);
static bool handle_program_special_insert(runstate_t *rs);
static bool handle_special_l1_state(runstate_t *rs, int8_t *digit_to_buffer);
static void handle_special_l2_state(runstate_t *rs);
static void handle_programming_state(runstate_t *rs);
static void handle_hotline_slot_state(runstate_t *rs);
static void handle_hotline_delay_state(runstate_t *rs);
static void load_hotline_config(hotline_config_t *config);
static void write_hotline_config(const hotline_config_t *config);
static uint16_t hotline_delay_ms(uint8_t delay_digit);
static bool wait_for_hotline_window(uint16_t delay_ms);
static void suppress_first_release_if_needed(runstate_t *rs, bool dial_pin_state);
static void maybe_dial_hotline(void);
static void dial_speed_dial_number(int8_t *speed_dial_digits, int8_t index, uint16_t dtmf_duration);
static void flush_pending_pauses(runstate_t *rs);
static void check_service_codes(runstate_t *rs);
static void handle_factory_reset(runstate_t *rs);
static bool dial_locked(runstate_t *rs);
static bool programming_locked(runstate_t *rs);
static bool pin_matches(runstate_t *rs, const uint8_t *pin);
static void clear_lock(runstate_t *rs);
static void handle_lock_code(runstate_t *rs, uint8_t requested_lock);
static void handle_session_unlock(runstate_t *rs);
static void wdt_timer_start(uint8_t delay);
static void start_sleep(void);
static void wdt_stop(void);
static bool in_pin_entry_mode(runstate_t *rs);

// Map speed dial numbers to memory locations
const int8_t _g_speed_dial_loc[] =
{
    0,
    -1 /* 1 - * */,
    -1 /* 2 - # */,
    -1 /* 3 - Redial */,
    1,
    2,
    3,
    4,
    5,
    6 
};

typedef struct
{
    int8_t code[5];
    uint8_t action;
} service_code_t;

#define SERVICE_FACTORY_RESET   0

static const service_code_t _g_service_codes[] =
{
    { { DIGIT_STAR, DIGIT_POUND, 0, DIGIT_POUND, DIGIT_STAR }, SERVICE_FACTORY_RESET },
};

int8_t EEMEM _g_speed_dial_eeprom[SPEED_DIAL_COUNT][SPEED_DIAL_SIZE] = { [0 ... (SPEED_DIAL_COUNT - 1)][0 ... SPEED_DIAL_SIZE - 1] = DIGIT_OFF };
hotline_config_t EEMEM _g_hotline_eeprom = { 0, 0, 5 };
uint8_t EEMEM _g_dtmf_duration_eeprom = 0;   // 0=DTMF_DURATION_MS_SHORT, 1=DTMF_DURATION_MS_LONG
uint8_t EEMEM _g_l1_hold_time_eeprom = 0;    // 0=SPECIAL_L1_HOLD_TIME_LONG, 1=SPECIAL_L1_HOLD_TIME_SHORT
uint8_t EEMEM _g_lock_pin_eeprom[3] = { DIGIT_OFF, DIGIT_OFF, DIGIT_OFF };
uint8_t EEMEM _g_lock_state_eeprom = 0;
runstate_t _g_run_state;

int main(void)
{
    runstate_t *rs = &_g_run_state;
    bool dial_pin_prev_state;
    rs->pending_pauses = 0;
    uint8_t lock_state = eeprom_read_byte(&_g_lock_state_eeprom);
    if (lock_state > LOCK_FULL) lock_state = LOCK_NONE;
    rs->lock_state = lock_state;
    init();

    // Wait for the decoupling capacitors to charge
    wdt_timer_start(SLEEP_128MS);
    start_sleep();
    wdt_stop();

    dtmf_init();

    // Local dial status variables 
    rs->state = STATE_DIAL;
    rs->dial_pin_state = true;
    rs->flags = F_NONE;
    rs->special_hold_state = STATE_DIAL;
    rs->suppress_first_release = false;
    rs->special_l1_latched = false;
    for (uint8_t i = 0; i < 5; i++)
        rs->service_code_buf[i] = DIGIT_OFF;
    rs->program_special_insert = false;
    rs->speed_dial_digit_index = 0;
    rs->speed_dial_index = 0;
    rs->hotline_slot_digit = DIGIT_OFF;

    uint8_t dtmf_dur_index = eeprom_read_byte(&_g_dtmf_duration_eeprom);
    if (dtmf_dur_index >= 2) dtmf_dur_index = 0;
    rs->dtmf_duration = (dtmf_dur_index == 0) ? DTMF_DURATION_MS_SHORT : DTMF_DURATION_MS_LONG;

    uint8_t l1_hold_index = eeprom_read_byte(&_g_l1_hold_time_eeprom);
    if (l1_hold_index >= 2) l1_hold_index = 0;
    rs->l1_hold_time = (l1_hold_index == 0) ? SLEEP_2S : SLEEP_1S;
    
    dial_pin_prev_state = true;
    
    for (uint8_t i = 0; i < SPEED_DIAL_SIZE; i++)
        rs->speed_dial_digits[i] = DIGIT_OFF;

//#define DEBUG
    #ifdef DEBUG
        // DEBUG: beep speed_dial_digit_index times to show redial buffer fill level
        {
            int8_t tmp[SPEED_DIAL_SIZE];
            read_speed_dial(SPEED_DIAL_REDIAL, tmp);
            uint8_t count = 0;
            while (count < SPEED_DIAL_SIZE && tmp[count] != DIGIT_OFF)
                count++;
            for (uint8_t i = 0; i < count; i++)
            {
                dtmf_generate_tone(DIGIT_BEEP_LOW, 80);
                sleep_ms(200);
            }
            sleep_ms(1000);
        }
    #endif

    maybe_dial_hotline();

    while (1)
    {
        rs->dial_pin_state = bit_is_set(PINB, PIN_DIAL);

        bool suppressed = rs->suppress_first_release;

        suppress_first_release_if_needed(rs, rs->dial_pin_state);
        if (rs->special_l1_latched && !rs->dial_pin_state)
        {
            dial_pin_prev_state = rs->dial_pin_state;
            continue;
        }
        // Skip if flag was set BEFORE clearing - prevents redial on first release
        if (suppressed)
        {
            dial_pin_prev_state = rs->dial_pin_state;
            continue;
        }

        if (dial_pin_prev_state != rs->dial_pin_state) 
        {
            if (!rs->dial_pin_state) 
            {
                // Dial just started
                if (!rs->special_l1_latched)
                    rs->flags |= F_DETECT_SPECIAL_L1;
                rs->flags &= ~F_DETECT_PAUSE; 
                rs->dialed_digit = 0;
 
                wdt_timer_start(SLEEP_64MS);
                start_sleep();
            }
            else 
            {
                // Disable SF detection (should be already disabled)
                rs->flags &= ~(F_DETECT_SPECIAL_L1 | F_DETECT_SPECIAL_L2);

                // Check that we detect a valid digit
                if (rs->dialed_digit <= 0 || rs->dialed_digit > 10)
                {
                    rs->dialed_digit = DIGIT_OFF;
    
                    if (rs->state == STATE_SPECIAL_L1)
                    {
                        // Erdtaste L1 - dial Erdtaste speed dial
                        wdt_stop();
                        dial_speed_dial_number(rs->speed_dial_digits, SPEED_DIAL_ERDTASTE, rs->dtmf_duration);
                        rs->state = STATE_DIAL;
                    }
                    else if (rs->state == STATE_SPECIAL_L2)
                    {
                        if (programming_locked(rs))
                        {
                            dtmf_generate_tone(DIGIT_TUNE_DESC, 200);
                            rs->state = STATE_DIAL;
                        }
                        else
                        {
                            // Erdtaste L2 - enter Erdtaste speed dial programming
                            wdt_stop();
                            rs->speed_dial_index = SPEED_DIAL_ERDTASTE;
                            rs->special_hold_state = STATE_PROGRAM_ERDTASTE;
                            clear_current_program_buffer(rs);
                            rs->state = STATE_PROGRAM_ERDTASTE;
                        }
                    }
                    else if (is_programming_state(rs->state))
                    {
                        // Erdtaste short press during programming - load redial into buffer
                        wdt_stop();
                        read_speed_dial(SPEED_DIAL_REDIAL, rs->speed_dial_digits);
                        load_into_current_slot(rs->speed_dial_digits, rs);
                        // Stay in current programming state
                    }        
                    else if (rs->state == STATE_HOTLINE_SLOT)
                    {
                        // Erdtaste short press during hotline slot selection - use Erdtaste slot as hotline
                        wdt_stop();
                        rs->hotline_slot_digit = DIGIT_OFF;  // no dial digit, special marker
                        rs->state = STATE_HOTLINE_DELAY;
                        dtmf_generate_tone(DIGIT_BEEP_LOW, rs->dtmf_duration);

                        hotline_config_t config;
                        config.enabled = 1;
                        config.slot_digit = DIGIT_OFF;  // special value meaning Erdtaste slot
                        config.delay_digit = 5;
                        write_hotline_config(&config);
                    }                                    
                    else if (rs->state == STATE_DIAL && !rs->suppress_first_release)
                    {
                        wdt_stop();
                        if (dial_locked(rs))
                        {
                            // Locked: Erdtaste dials the Erdtaste speed-dial slot directly
                            dial_speed_dial_number(rs->speed_dial_digits, SPEED_DIAL_ERDTASTE, rs->dtmf_duration);
                        }
                        else
                        {
                            // Normal: Erdtaste triggers redial
                            read_speed_dial(SPEED_DIAL_REDIAL, rs->speed_dial_digits);
                            dial_speed_dial_number(rs->speed_dial_digits, SPEED_DIAL_REDIAL, rs->dtmf_duration);
                        }
                    }
                    else
                    {
                        rs->suppress_first_release = false;
                        wdt_timer_start(SLEEP_64MS);
                        start_sleep();
                    }
                }
                else 
                {
                    // Got a valid digit - process it            
#ifdef NZ_DIAL
                    // NZPO Phones only. 0 is same as GPO but 1-9 are reversed.
                    rs->dialed_digit = (10 - rs->dialed_digit);
#else
                    if (rs->dialed_digit == 10)
                        rs->dialed_digit = 0; // 10 pulses => 0
#endif
                    wdt_timer_start(SLEEP_128MS);
                    start_sleep();
                    wdt_stop();

                    process_dialed_digit(rs);
                }

                rs->special_l1_latched = false;
            }    
        } 
        else 
        {
            if (rs->dial_pin_state) 
            {
                // Rotary dial at the rest position
                // Reset all variables
                rs->state = STATE_DIAL;
                rs->special_hold_state = STATE_DIAL;
                rs->flags &= ~(F_DETECT_SPECIAL_L1 | F_DETECT_SPECIAL_L2);
                rs->dialed_digit = DIGIT_OFF;
            }
        }

        dial_pin_prev_state = rs->dial_pin_state;

        // Don't power down if special function detection is active        
        if (rs->flags & F_DETECT_SPECIAL_L1)
        {
            rs->flags &= ~F_WDT_AWAKE;
            // Put MCU to sleep - to be awoken either by pin interrupt or WDT
            wdt_timer_start(rs->l1_hold_time);
            start_sleep();

            // Special function mode detected?
            if (rs->flags & F_WDT_AWAKE)
            {
                // SF mode detected
                rs->flags &= ~F_WDT_AWAKE;
                rs->special_hold_state = rs->state;
                if (is_programming_state(rs->state))
                    rs->program_special_insert = true;
                rs->state = STATE_SPECIAL_L1;
                rs->flags &= ~F_DETECT_SPECIAL_L1;
                if (!programming_locked(rs))
                    rs->flags |= F_DETECT_SPECIAL_L2;
                else
                {
                    rs->special_l1_latched = true;
                    wdt_stop();
                }

                // Indicate that we entered L1 SF mode with short beep
                dtmf_generate_tone(DIGIT_BEEP_LOW, 200);
            }
            else
            {
                // Released before the first timeout: cancel special detection so a
                // short press can be handled as redial on the next loop.
                rs->flags &= ~F_DETECT_SPECIAL_L1;
            }
        }
        else if (rs->flags & F_DETECT_SPECIAL_L2)
        {
            rs->flags &= ~F_WDT_AWAKE;
            // Put MCU to sleep - to be awoken either by pin interrupt or WDT
            wdt_timer_start(SPECIAL_L2_HOLD_TIME);
            start_sleep();

            if (rs->flags & F_WDT_AWAKE)
            {
                // SF mode detected
                rs->flags &= ~F_WDT_AWAKE;
                rs->state = STATE_SPECIAL_L2;
                rs->flags &= ~F_DETECT_SPECIAL_L2;

                // Indicate that we entered L2 SF mode with asc tone
                dtmf_generate_tone(DIGIT_TUNE_ASC, 200);
            }
            else
            {
                // Released before the second timeout: cancel special detection.
                rs->flags &= ~F_DETECT_SPECIAL_L2;
            }
        }        
        else if (rs->flags & F_DETECT_PAUSE)
        {
            rs->flags &= ~F_WDT_AWAKE;
            wdt_timer_start(PAUSE_DETECT_HOLD_TIME);
            start_sleep();

            if (rs->flags & F_WDT_AWAKE)
            {
                rs->flags &= ~F_WDT_AWAKE;
                if (rs->pending_pauses < 2)
                {
                    rs->pending_pauses++;
                    if (rs->pending_pauses >= 2)
                    {
                        rs->flags &= ~F_DETECT_PAUSE;  // max pauses reached, stop
                        wdt_stop();
                    }
                        
                }
            }
            else
            {
                // Woken by pin interrupt (dial activity) - cancel pause detection
                rs->flags &= ~F_DETECT_PAUSE;
            }
        }
        else
        {
            // Don't need timer - sleep to power down mode
            set_sleep_mode(SLEEP_MODE_PWR_DOWN);
            sleep_mode();
        }
    }

    return 0;
}

// New speed-dial buffer helpers: keep EEPROM layout and END-marker behavior centralized.
static void read_speed_dial(uint8_t slot, int8_t *buf)
{
    if (slot < SPEED_DIAL_COUNT)
        eeprom_read_block(buf, &_g_speed_dial_eeprom[slot][0], SPEED_DIAL_SIZE);
}

static void write_speed_dial(uint8_t slot, int8_t *buf)
{
    if (slot < SPEED_DIAL_COUNT)
        eeprom_update_block(buf, &_g_speed_dial_eeprom[slot][0], SPEED_DIAL_SIZE);
}

static uint8_t find_end(int8_t *buf)
{
    uint8_t index = 0;

    while (index < SPEED_DIAL_SIZE && buf[index] >= 0)
        index++;

    return index;
}

static bool is_programming_state(uint8_t state)
{
    return state == STATE_PROGRAM_SD || state == STATE_PROGRAM_ERDTASTE;
}

static uint8_t current_program_slot(runstate_t *rs)
{
    if (rs->state == STATE_PROGRAM_ERDTASTE || rs->special_hold_state == STATE_PROGRAM_ERDTASTE)
        return SPEED_DIAL_ERDTASTE;

    return rs->speed_dial_index;
}

static void clear_current_program_buffer(runstate_t *rs)
{
    for (uint8_t i = 0; i < SPEED_DIAL_SIZE; i++)
        rs->speed_dial_digits[i] = DIGIT_OFF;

    rs->speed_dial_digit_index = 0;
}

static void load_into_current_slot(int8_t *src, runstate_t *rs)
{
    for (uint8_t i = 0; i < SPEED_DIAL_SIZE; i++)
        rs->speed_dial_digits[i] = src[i];

    rs->speed_dial_digit_index = find_end(rs->speed_dial_digits);
    write_speed_dial(current_program_slot(rs), rs->speed_dial_digits);

    for (uint8_t i = 0; i < rs->speed_dial_digit_index; i++)
    {
        dtmf_generate_tone(DIGIT_BEEP_LOW, rs->dtmf_duration);
        sleep_ms(rs->dtmf_duration);
    }
}

static void store_digit_to_redial(runstate_t *rs, int8_t digit)
{
    if (rs->speed_dial_digit_index < SPEED_DIAL_SIZE)
    {
        rs->speed_dial_digits[rs->speed_dial_digit_index] = digit;
        rs->speed_dial_digit_index++;
        write_speed_dial(SPEED_DIAL_REDIAL, rs->speed_dial_digits);
        if (digit != DIGIT_PAUSE)
        {
            rs->pending_pauses = 0;
            rs->flags |= F_DETECT_PAUSE;
        }
    }
}

static void push_to_service_buffer(runstate_t *rs, int8_t digit)
{
    for (uint8_t i = 0; i < 4; i++) {
        rs->service_code_buf[i] = rs->service_code_buf[i + 1];
    }
    rs->service_code_buf[4] = digit;
}

static void check_service_codes(runstate_t *rs)
{
    for (uint8_t i = 0; i < sizeof(_g_service_codes) / sizeof(_g_service_codes[0]); i++)
    {
        if (memcmp(rs->service_code_buf, _g_service_codes[i].code, 5) == 0)
        {
            if (_g_service_codes[i].action == SERVICE_FACTORY_RESET)
                handle_factory_reset(rs);
                
            // Consume the code
            memset(rs->service_code_buf, DIGIT_OFF, 5); 
            return;                      
        }
    }

    // Check full lock/permanent unlock - *#PPP
    if (rs->service_code_buf[0] == DIGIT_STAR && 
        rs->service_code_buf[1] == DIGIT_POUND &&
        rs->service_code_buf[2] >= 0 && rs->service_code_buf[2] <= 9 &&
        rs->service_code_buf[3] >= 0 && rs->service_code_buf[3] <= 9 &&
        rs->service_code_buf[4] >= 0 && rs->service_code_buf[4] <= 9)
    {
        handle_lock_code(rs, LOCK_FULL);
        // Consume the code
        memset(rs->service_code_buf, DIGIT_OFF, 5);
        clear_current_program_buffer(rs);
    }

    // Check programming lock/permanent unlock - **PPP
    if (rs->service_code_buf[0] == DIGIT_STAR &&
        rs->service_code_buf[1] == DIGIT_STAR &&
        rs->service_code_buf[2] >= 0 && rs->service_code_buf[2] <= 9 &&
        rs->service_code_buf[3] >= 0 && rs->service_code_buf[3] <= 9 &&
        rs->service_code_buf[4] >= 0 && rs->service_code_buf[4] <= 9)
    {
        handle_lock_code(rs, LOCK_PROGRAM);
        // Consume the code
        memset(rs->service_code_buf, DIGIT_OFF, 5);
        clear_current_program_buffer(rs);
    }

    // Check session unlock - ##PPP
    if (rs->service_code_buf[0] == DIGIT_POUND &&
        rs->service_code_buf[1] == DIGIT_POUND &&
        rs->service_code_buf[2] >= 0 && rs->service_code_buf[2] <= 9 &&
        rs->service_code_buf[3] >= 0 && rs->service_code_buf[3] <= 9 &&
        rs->service_code_buf[4] >= 0 && rs->service_code_buf[4] <= 9)
    {
        handle_session_unlock(rs);
        // Consume the code
        memset(rs->service_code_buf, DIGIT_OFF, 5);
        clear_current_program_buffer(rs);
    }

}

static void handle_factory_reset(runstate_t *rs)
{
    if (programming_locked(rs))
    {
        dtmf_generate_tone(DIGIT_TUNE_DESC, 200);
        return;
    }
    dtmf_generate_tone(DIGIT_TUNE_DESC, 800);
    for (uint16_t i = 0; i < 512; i++)
    {
        wdt_reset();
        eeprom_write_byte((uint8_t*)i, 0xFF);
    }
    rs->dtmf_duration = DTMF_DURATION_MS_SHORT;
    rs->l1_hold_time = SLEEP_2S;
    rs->lock_state = LOCK_NONE;
    for (uint8_t i = 0; i < 5; i++)
        rs->service_code_buf[i] = DIGIT_OFF;
    dtmf_generate_tone(DIGIT_TUNE_ASC, 400);
}

static bool dial_locked(runstate_t *rs)
{
    return rs->lock_state == LOCK_FULL;
}

static bool programming_locked(runstate_t *rs)
{
    return rs->lock_state == LOCK_PROGRAM || rs->lock_state == LOCK_FULL;
}

static bool pin_matches(runstate_t *rs, const uint8_t *pin)
{
    return pin[0] == rs->service_code_buf[2] &&
           pin[1] == rs->service_code_buf[3] &&
           pin[2] == rs->service_code_buf[4];
}

static void clear_lock(runstate_t *rs)
{
    eeprom_write_byte(&_g_lock_pin_eeprom[0], (uint8_t)DIGIT_OFF);
    eeprom_write_byte(&_g_lock_pin_eeprom[1], (uint8_t)DIGIT_OFF);
    eeprom_write_byte(&_g_lock_pin_eeprom[2], (uint8_t)DIGIT_OFF);
    eeprom_write_byte(&_g_lock_state_eeprom, LOCK_NONE);
    rs->lock_state = LOCK_NONE;
}

static void handle_lock_code(runstate_t *rs, uint8_t requested_lock)
{
    uint8_t pin[3];
    eeprom_read_block(pin, &_g_lock_pin_eeprom, 3);

    if (pin[0] == (uint8_t)DIGIT_OFF)
    {
        // No PIN set - set PIN and requested lock
        eeprom_write_byte(&_g_lock_pin_eeprom[0], rs->service_code_buf[2]);
        eeprom_write_byte(&_g_lock_pin_eeprom[1], rs->service_code_buf[3]);
        eeprom_write_byte(&_g_lock_pin_eeprom[2], rs->service_code_buf[4]);
        eeprom_write_byte(&_g_lock_state_eeprom, requested_lock);
        rs->lock_state = requested_lock;
        dtmf_generate_tone(DIGIT_TUNE_ASC, 400);
    }
    else if (pin_matches(rs, pin))
    {
        // Correct PIN - permanently unlock and clear
        clear_lock(rs);
        dtmf_generate_tone(DIGIT_TUNE_DESC, 400);
    }
    // Wrong PIN - silence
}

static void handle_session_unlock(runstate_t *rs)
{
    uint8_t pin[3];
    eeprom_read_block(pin, &_g_lock_pin_eeprom, 3);

    if (pin[0] == (uint8_t)DIGIT_OFF)
        return;

    if (pin_matches(rs, pin))
    {
        rs->lock_state = LOCK_NONE;
        dtmf_generate_tone(DIGIT_TUNE_ASC, 400);
    }
    // Wrong PIN - silence
}

static void clear_speed_dial_slot(runstate_t *rs, uint8_t slot_index)
{
    clear_current_program_buffer(rs);
    write_speed_dial(slot_index, rs->speed_dial_digits);
}

static void append_digit_to_slot(runstate_t *rs, int8_t digit, uint8_t tone)
{
    if (rs->speed_dial_digit_index < SPEED_DIAL_SIZE)
    {
        rs->speed_dial_digits[rs->speed_dial_digit_index] = digit;
        rs->speed_dial_digit_index++;

        write_speed_dial(current_program_slot(rs), rs->speed_dial_digits);
        dtmf_generate_tone(tone, rs->dtmf_duration);
    }
    else
    {
        // Memory full
        dtmf_generate_tone(DIGIT_TUNE_DESC, 800);
        rs->state = STATE_DIAL;
    }
}

static void flush_pending_pauses(runstate_t *rs)
{
    while (rs->pending_pauses > 0 && rs->speed_dial_digit_index < SPEED_DIAL_SIZE)
    {
        store_digit_to_redial(rs, DIGIT_PAUSE);
        rs->pending_pauses--;
    }
    rs->pending_pauses = 0;
}

// PIN entry mode is active when the first two digits dialed this call were both
// * or # (entered via L1) and the three PIN digits have not yet all been received.
// The check fires before the index increment, so the three PIN digits arrive with
// index at 2, 3 and 4 respectively - hence the upper bound is <= 4, not <= 5.
static bool in_pin_entry_mode(runstate_t *rs)
{
    return rs->speed_dial_digit_index >= 2
        && rs->speed_dial_digit_index <= 4
        && (rs->speed_dial_digits[0] == DIGIT_STAR || rs->speed_dial_digits[0] == DIGIT_POUND)
        && (rs->speed_dial_digits[1] == DIGIT_STAR || rs->speed_dial_digits[1] == DIGIT_POUND);
}

static void handle_dial_state(runstate_t *rs, int8_t *digit_to_buffer)
{
    *digit_to_buffer = rs->dialed_digit;
    if (!dial_locked(rs))
    {
        dtmf_generate_tone(rs->dialed_digit, rs->dtmf_duration);
        flush_pending_pauses(rs);
        store_digit_to_redial(rs, *digit_to_buffer);
    }
    else if (in_pin_entry_mode(rs))
    {
        // Mid-unlock sequence: feed digit to service buffer via digit_to_buffer,
        // increment index so the window advances, but do not dial anything.
        rs->speed_dial_digits[rs->speed_dial_digit_index] = rs->dialed_digit;
        rs->speed_dial_digit_index++;
        dtmf_generate_tone(DIGIT_BEEP_LOW, rs->dtmf_duration);
    }
    else if (_g_speed_dial_loc[rs->dialed_digit] >= 0)
    {
        // Locked direct speed-dial: digits 4-9 and 0 fire their slot immediately.
        dial_speed_dial_number(rs->speed_dial_digits, _g_speed_dial_loc[rs->dialed_digit], rs->dtmf_duration);
        *digit_to_buffer = DIGIT_OFF;  // do not feed service buffer
    }
    else
    {
        // Digits 1, 2, 3 with no PIN entry mode active: no speed-dial slot,
        // play descending tone as "not available" feedback, same as empty slot.
        dtmf_generate_tone(DIGIT_TUNE_DESC, 400);
        *digit_to_buffer = DIGIT_OFF;
    }
}

static bool handle_program_special_insert(runstate_t *rs)
{
    if (!rs->program_special_insert)
        return false;

    if (rs->dialed_digit == L2_STAR || rs->dialed_digit == L2_POUND)
    {
        append_digit_to_slot(rs, (rs->dialed_digit == L2_STAR) ? DIGIT_STAR : DIGIT_POUND, DIGIT_BEEP_LOW);
        rs->state = (rs->special_hold_state == STATE_PROGRAM_ERDTASTE) ? STATE_PROGRAM_ERDTASTE : STATE_PROGRAM_SD;
        rs->program_special_insert = false;
        return true;
    }

    if (rs->dialed_digit == L2_REDIAL)
    {
        // Insert pause on dial 3
        append_digit_to_slot(rs, DIGIT_PAUSE, DIGIT_BEEP_LOW);
        rs->state = (rs->special_hold_state == STATE_PROGRAM_ERDTASTE) ? STATE_PROGRAM_ERDTASTE : STATE_PROGRAM_SD;
        rs->program_special_insert = false;
        return true;
    }

    if (_g_speed_dial_loc[rs->dialed_digit] >= 0)
    {
        read_speed_dial(_g_speed_dial_loc[rs->dialed_digit], rs->speed_dial_digits);
        load_into_current_slot(rs->speed_dial_digits, rs);
        rs->state = (rs->special_hold_state == STATE_PROGRAM_ERDTASTE) ? STATE_PROGRAM_ERDTASTE : STATE_PROGRAM_SD;
        rs->program_special_insert = false;
        return true;
    }

    rs->program_special_insert = false;
    return false;
}

static bool handle_special_l1_state(runstate_t *rs, int8_t *digit_to_buffer)
{
    if (handle_program_special_insert(rs))
        return true;

    if (rs->dialed_digit == L2_STAR)
    {
        // SF 1-*
        if (dial_locked(rs))
            dtmf_generate_tone(DIGIT_BEEP_LOW, rs->dtmf_duration);
        else
            dtmf_generate_tone(DIGIT_STAR, rs->dtmf_duration);
        *digit_to_buffer = DIGIT_STAR;
        store_digit_to_redial(rs, DIGIT_STAR);
        rs->state = STATE_DIAL;
    }
    else if (rs->dialed_digit == L2_POUND)
    {
        // SF 2-#
        if (dial_locked(rs))
            dtmf_generate_tone(DIGIT_BEEP_LOW, rs->dtmf_duration);
        else
            dtmf_generate_tone(DIGIT_POUND, rs->dtmf_duration);
        *digit_to_buffer = DIGIT_POUND;
        store_digit_to_redial(rs, DIGIT_POUND);
        rs->state = STATE_DIAL;
    }
    else if (rs->dialed_digit == L2_REDIAL)
    {
        // SF 3 (Redial) - not available when dial is locked
        if (!dial_locked(rs))
        {
            wdt_stop();
            dial_speed_dial_number(rs->speed_dial_digits, SPEED_DIAL_REDIAL, rs->dtmf_duration);
        }
        else
        {
            dtmf_generate_tone(DIGIT_TUNE_DESC, 400);
        }
        rs->state = STATE_DIAL;
    }
    else if (_g_speed_dial_loc[rs->dialed_digit] >= 0)
    {
        // Call speed dial number
        dial_speed_dial_number(rs->speed_dial_digits, _g_speed_dial_loc[rs->dialed_digit], rs->dtmf_duration);
        rs->state = STATE_DIAL;
    }

    return false;
}

static void handle_special_l2_state(runstate_t *rs)
{
    if (programming_locked(rs))
    {
        dtmf_generate_tone(DIGIT_TUNE_DESC, 200);
        rs->state = STATE_DIAL;
        return;
    }

    // Digit 1 - cycle DTMF duration (80ms / 200ms)
    if (rs->dialed_digit == L2_STAR)
    {
        uint8_t index = eeprom_read_byte(&_g_dtmf_duration_eeprom);
        if (index >= 2) index = 0;
        if (index >= 1) index = 0; else index++;
        eeprom_write_byte(&_g_dtmf_duration_eeprom, index);
        rs->dtmf_duration = (index == 0) ? DTMF_DURATION_MS_SHORT : DTMF_DURATION_MS_LONG;
        uint16_t beep_dur = (index == 0) ? 80 : 300;
        uint16_t gap = (index == 0) ? 150 : 300;
        dtmf_generate_tone(DIGIT_BEEP, beep_dur);
        sleep_ms(gap);
        dtmf_generate_tone(DIGIT_BEEP, beep_dur);
        rs->state = STATE_DIAL;
        return;
    }

    // Digit 2 - cycle L1 hold time (2s / 1s)
    if (rs->dialed_digit == L2_POUND)
    {
        uint8_t index = eeprom_read_byte(&_g_l1_hold_time_eeprom);
        if (index >= 2) index = 0;
        if (index >= 1) index = 0; else index++;
        eeprom_write_byte(&_g_l1_hold_time_eeprom, index);
        rs->l1_hold_time = (index == 0) ? SLEEP_2S : SLEEP_1S;
        uint16_t beep_dur = (index == 0) ? 300 : 80;
        uint16_t gap = (index == 0) ? 300 : 150;
        dtmf_generate_tone(DIGIT_BEEP, beep_dur);
        sleep_ms(gap);
        dtmf_generate_tone(DIGIT_BEEP, beep_dur);
        rs->state = STATE_DIAL;
        return;
    }

    // Clear hotline - hold 3 then hold 0 (before entering hotline slot)
    if (rs->program_special_insert && rs->dialed_digit == 0 && rs->special_hold_state == STATE_HOTLINE_SLOT)
    {
        hotline_config_t config;

        load_hotline_config(&config);
        config.enabled = 0;
        config.slot_digit = DIGIT_OFF;
        config.delay_digit = DIGIT_OFF;
        write_hotline_config(&config);
        dtmf_generate_tone(DIGIT_TUNE_DESC, 800);
        rs->hotline_slot_digit = DIGIT_OFF;
        rs->state = STATE_DIAL;
        rs->program_special_insert = false;
        return;
    }

    // Clear SD or Erdtaste slot - hold 2nd beep then dial 0
    if (rs->program_special_insert && rs->dialed_digit == 0 && is_programming_state(rs->special_hold_state))
    {
        clear_speed_dial_slot(rs, rs->speed_dial_index);
        dtmf_generate_tone(DIGIT_TUNE_DESC, 400);
        rs->state = STATE_DIAL;
        rs->program_special_insert = false;
        return;
    }

    if (rs->dialed_digit == L2_REDIAL)
    {
        rs->special_hold_state = STATE_HOTLINE_SLOT;
        rs->state = STATE_HOTLINE_SLOT;
        rs->program_special_insert = true;
    }
    else if (_g_speed_dial_loc[rs->dialed_digit] >= 0)
    {
        rs->special_hold_state = STATE_PROGRAM_SD;
        rs->speed_dial_index = _g_speed_dial_loc[rs->dialed_digit];
        clear_current_program_buffer(rs);
        rs->state = STATE_PROGRAM_SD;
        rs->program_special_insert = true;
    }
    else
    {
        rs->state = STATE_DIAL;
    }
}

static void handle_programming_state(runstate_t *rs)
{
    append_digit_to_slot(rs, rs->dialed_digit, DIGIT_BEEP_LOW);
}

static void handle_hotline_slot_state(runstate_t *rs)
{
    if (_g_speed_dial_loc[rs->dialed_digit] >= 0)
    {
        // Select the stored number that will become the hotline number
        rs->hotline_slot_digit = rs->dialed_digit;
        rs->state = STATE_HOTLINE_DELAY;
        dtmf_generate_tone(DIGIT_BEEP_LOW, rs->dtmf_duration);

        // Save the default delay immediately unless the caller overrides it
        // with an explicit delay digit afterwards.
        hotline_config_t config;

        config.enabled = 1;
        config.slot_digit = rs->hotline_slot_digit;
        config.delay_digit = 5;
        write_hotline_config(&config);
    }
    else
    {
        rs->state = STATE_DIAL;
    }
}

static void handle_hotline_delay_state(runstate_t *rs)
{
    hotline_config_t config;

    // Hotline delay accepts 0..9:
    // 0 = 0 seconds, 1..9 = 1..9 seconds.
    if (rs->dialed_digit <= 9)
    {
        config.enabled = 1;
        config.slot_digit = rs->hotline_slot_digit;
        config.delay_digit = rs->dialed_digit;
        write_hotline_config(&config);
        dtmf_generate_tone(DIGIT_TUNE_ASC, 400);
    }
    else
    {
        dtmf_generate_tone(DIGIT_TUNE_DESC, 400);
    }

    rs->hotline_slot_digit = DIGIT_OFF;
    rs->state = STATE_DIAL;
}

static void process_dialed_digit(runstate_t *rs)
{
    int8_t digit_to_buffer = DIGIT_OFF;
    
    if (rs->state == STATE_DIAL)
    {
        handle_dial_state(rs, &digit_to_buffer);
    }
    else if (rs->state == STATE_SPECIAL_L1)
    {
        if (handle_special_l1_state(rs, &digit_to_buffer))
            return;
    }
    else if (rs->state == STATE_SPECIAL_L2)
    {
        handle_special_l2_state(rs);
    }
    else if (is_programming_state(rs->state))
    {
        handle_programming_state(rs);
    }
    else if (rs->state == STATE_HOTLINE_SLOT)
    {
        handle_hotline_slot_state(rs);
    }
    else if (rs->state == STATE_HOTLINE_DELAY)
    {
        handle_hotline_delay_state(rs);
    }

    if (rs->speed_dial_digit_index <= 5)
    {
        if (digit_to_buffer != DIGIT_OFF)
        {
            push_to_service_buffer(rs, digit_to_buffer);
            check_service_codes(rs);
        }
    }
}

// Dial speed dial number (it erases current SD number in the global structure)
static void dial_speed_dial_number(int8_t *speed_dial_digits, int8_t index, uint16_t dtmf_duration)
{
    if (index >= 0 && index < SPEED_DIAL_COUNT)
    {
        read_speed_dial(index, speed_dial_digits);

        // Check for empty speed dial  
        bool empty = true;
        for (uint8_t i = 0; i < SPEED_DIAL_SIZE; i++)
        {
            if (speed_dial_digits[i] >= 0)
            {
                empty = false;
                break;
            }
        }

        // Beep to indicate speed dial was empty
        if (empty)
        {
            dtmf_generate_tone(DIGIT_TUNE_DESC, 400);
            return;
        }

        for (uint8_t i = 0; i < SPEED_DIAL_SIZE; i++)
        {
            // Skip invalid digits (negative or > 11, but allow DIGIT_PAUSE=12)
            if (speed_dial_digits[i] < 0 || (speed_dial_digits[i] > DIGIT_POUND && speed_dial_digits[i] != DIGIT_PAUSE))
                continue;

            // Handle pause
            if (speed_dial_digits[i] == DIGIT_PAUSE)
            {
                sleep_ms(DTMF_PAUSE_MS);
                continue;
            }

            // Send the tone
            dtmf_generate_tone(speed_dial_digits[i], dtmf_duration);

            // Normal pause after tone (always added)
            sleep_ms(dtmf_duration);
        }
    }
}

static void load_hotline_config(hotline_config_t *config)
{
    eeprom_read_block(config, &_g_hotline_eeprom, sizeof(*config));
}

static void write_hotline_config(const hotline_config_t *config)
{
    eeprom_update_block(config, &_g_hotline_eeprom, sizeof(*config));
}

static uint16_t hotline_delay_ms(uint8_t delay_digit)
{
    static const uint16_t delay_table[] =
    {
        500,
        1000,
        2000,
        3000,
        4000,
        5000,
        6000,
        7000,
        8000,
        9000
    };

    if (delay_digit > 9)
        return 0;

    return delay_table[delay_digit];
}

static bool wait_for_hotline_window(uint16_t delay_ms)
{
    uint16_t elapsed = 0;

    while (elapsed < delay_ms)
    {
        if (!bit_is_set(PINB, PIN_DIAL))
        {
            return false;
        }

        wdt_timer_start(SLEEP_64MS);
        start_sleep();
        wdt_stop();
        elapsed += 64;
    }

    return true;
}

static void suppress_first_release_if_needed(runstate_t *rs, bool dial_pin_state)
{
    if (!rs->suppress_first_release)
        return;

    rs->state = STATE_DIAL;
    rs->flags &= ~(F_DETECT_SPECIAL_L1 | F_DETECT_SPECIAL_L2);
    rs->dialed_digit = DIGIT_OFF;

    // Only the dial/Erdtaste line should release this latch.
    if (dial_pin_state)
        rs->suppress_first_release = false;
}

static void maybe_dial_hotline(void)
{
    hotline_config_t config;
    int8_t hotline_index;
    uint16_t delay_ms;

    load_hotline_config(&config);

    if (config.enabled != 1)
        return;

    if (config.slot_digit == (uint8_t)DIGIT_OFF)
        hotline_index = SPEED_DIAL_ERDTASTE;
    else
    {
        if (config.slot_digit >= sizeof(_g_speed_dial_loc) / sizeof(_g_speed_dial_loc[0]))
            return;
        hotline_index = _g_speed_dial_loc[config.slot_digit];
        if (hotline_index < 0)
            return;
    }

    delay_ms = hotline_delay_ms(config.delay_digit);

    // Check: Button held AT boot (before power-on)
    if (!bit_is_set(PINB, PIN_DIAL))
    {
        _g_run_state.suppress_first_release = true;
        return;  // hotline canceled
    }

    // Wait for delay window (0-9 seconds based on config)
    if (!wait_for_hotline_window(delay_ms))
    {
        // Released before delay window - cancel
        return;
    }

    // Held through delay - dial hotline
    wdt_stop();
    uint8_t dur_idx = eeprom_read_byte(&_g_dtmf_duration_eeprom);
    if (dur_idx >= 2) dur_idx = 0;
    uint16_t dtmf_duration = (dur_idx == 0) ? DTMF_DURATION_MS_SHORT : DTMF_DURATION_MS_LONG;
    dial_speed_dial_number(_g_run_state.speed_dial_digits, hotline_index, dtmf_duration);
    _g_run_state.suppress_first_release = true;
}

static void init(void)
{
    // Program clock prescaller to divide + frequency by 1
    // Write CLKPCE 1 and other bits 0    
    CLKPR = _BV(CLKPCE);

    // Write prescaler value with CLKPCE = 0
    CLKPR = 0;

    // Enable pull-ups
    PORTB |= (_BV(PIN_DIAL) | _BV(PIN_PULSE));

    // Disable unused modules to save power
    PRR = _BV(PRTIM1) | _BV(PRUSI) | _BV(PRADC);
    ACSR = _BV(ACD);

    // Configure pin change interrupt
    MCUCR = _BV(ISC01) | _BV(ISC00);         // Set INT0 for falling edge detection
    GIMSK = _BV(INT0) | _BV(PCIE);           // Added INT0
    PCMSK = _BV(PIN_DIAL) | _BV(PIN_PULSE);

    // Enable interrupts
    sei();                              
}

static void wdt_timer_start(uint8_t delay)
{
    wdt_reset();
    cli();
    MCUSR = 0x00;
    WDTCR |= _BV(WDCE) | _BV(WDE);
    switch (delay)
    {
        case SLEEP_64MS:
            WDTCR = _BV(WDIE) | _BV(WDP1);
            break;
        case SLEEP_128MS:
            WDTCR = _BV(WDIE) | _BV(WDP1) | _BV(WDP0);
            break;
        case SLEEP_1S:
            WDTCR = _BV(WDIE) | _BV(WDP0) | _BV(WDP2); // 1024ms
            break;
        case SLEEP_2S:
            WDTCR = _BV(WDIE) | _BV(WDP0) | _BV(WDP1) | _BV(WDP2); // 2048ms
            break;
    }
    sei();
}

static void wdt_stop(void)
{
    wdt_reset();
    cli();
    MCUSR = 0x00;
    WDTCR |= _BV(WDCE) | _BV(WDE);
    WDTCR = 0x00;
    sei();
}

static void start_sleep(void)
{
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();                          // stop interrupts to ensure the BOD timed sequence executes as required
    sleep_enable();
    sleep_bod_disable();            // disable brown-out detection (good for 20-25µA)
    sei();                          // ensure interrupts enabled so we can wake up again
    sleep_cpu();                    // go to sleep
    sleep_disable();                // wake up here
}

// Handler for external interrupt on INT0 (PB2, pin 7)
ISR(INT0_vect)
{
    if (!_g_run_state.dial_pin_state)
    {
        // Disabling SF detection
         _g_run_state.flags &= ~(F_DETECT_SPECIAL_L1 | F_DETECT_SPECIAL_L2);
        // A pulse just started
        _g_run_state.dialed_digit++;
        wdt_reset();
    }
}

// Interrupt initiated by pin change on any enabled pin
ISR(PCINT0_vect)
{
}

// Handler for any unspecified 'bad' interrupts
ISR(BADISR_vect)
{
    // Do nothing, just wake up MCU
}

ISR(WDT_vect)
{
    _g_run_state.flags |= F_WDT_AWAKE;
}
