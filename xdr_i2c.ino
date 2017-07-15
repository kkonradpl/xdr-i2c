/*
 *  XDR-I2C 2016-07-24
 *  Copyright (C) 2012-2016  Konrad Kosmatka
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 3
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */
#include <Arduino.h>
#include <avr/pgmspace.h>
#include "I2cMaster.h"
#include "xdr_f1hd.h"
#include "filters.h"
#include "align.h"

/* If you have an IR diode for auto power-up, change this to 1 */
#define IR 0

/* If you have a transistor for auto power-up, change this to 1 */
#define POWER 0

/* Delay between tuner power up and XDR-I2C start in seconds */
#define SLEEP_TIME 6

/* Reset tuner before trying to power it up (for IR/POWER) */
#define IR_POWER_RESET 0

/* Delay after antenna switch in miliseconds */
#define ANTENNA_SWITCH_DELAY 30

/* Automatic rotator stop after specified time in seconds */
#define ROTATOR_TIMEOUT 90

/* Rotator delay between direction change in miliseconds */
#define ROTATOR_DELAY 1000

/* Maximum audio output level (0~2047) */
#define MAX_VOLUME 2047

/* Send DSP initialization data on start (for tuners without stock controller) */
#define INIT 0

/* Pinout */
#define RDS_PIN      2
#define IR_PIN       3
#define POWER_PIN    4
#define RESET_PIN    5
#define ROT_CW_PIN   6
#define ROT_CCW_PIN  7
#define ANT_A_PIN    8
#define ANT_B_PIN    9
#define ANT_C_PIN   10
#define ANT_D_PIN   11
#define BUTTON_PIN  13
#define SDA_PIN     A4
#define SCL_PIN     A5

TwiMaster i2c(false);

/* TEF6730 IF */
uint8_t CONTROL = 0x00;
uint16_t PLL;
uint8_t DAA = 0x00;
uint8_t AGC = 0xC8;
uint8_t BAND;

/* Serial port */
#define SERIAL_PORT_SPEED  115200
#define SERIAL_BUFFER_SIZE 16

/* Signal level & squelch */
#define TIMER_INTERVAL      66
#define SIGNAL_SAMPLE_COUNT  3
#define SQUELCH_TIMEOUT     10
float level[SIGNAL_SAMPLE_COUNT];
bool stereo[SIGNAL_SAMPLE_COUNT];
uint8_t level_fast_countdown;
bool level_fast = false;
uint16_t sampling_custom = 0;
int8_t squelch_threshold = 0;
uint8_t squelch_state = 0;

/* RDS */
#define RDS_SYNC_WAIT 10
/* PI_BUFFER_SIZE must be a multiple of 8. */
#define PI_BUFFER_SIZE 64
#define BUFF_RESET_FREQ_OFFSET 20
uint8_t pi_buffer_fill = 0;
uint8_t pi_pos = 0;
int8_t pi_state = -1;
uint32_t last_rds_reset = 0;

/* Scan */
bool scan_flag = false;
uint32_t scan_start = 0;
uint32_t scan_end = 0;
uint8_t scan_step = 0;
uint8_t scan_filter = 0;

/* Antenna switch */
const uint8_t ANT[] = {ANT_A_PIN, ANT_B_PIN, ANT_C_PIN, ANT_D_PIN};
const uint8_t ANT_n = sizeof(ANT)/sizeof(uint8_t);
uint8_t current_ant = 0;

/* Other */
#define BUTTON_DEBOUNCE 50
#define ST_THRESHOLD 0x052
uint8_t mode;
uint32_t current_freq = 87500;
int8_t current_filter = -1; // current FIR filter (-1 is adaptive)
uint16_t volume = MAX_VOLUME; // audio volume control
bool forced_mono = false;
int8_t rotator_req = -1;

#define MODE_FM 0
#define MODE_AM 1

#define LEVEL_SLOW 0
#define LEVEL_FAST 1

#define ROTATION_OFF 0
#define ROTATION_CW  1
#define ROTATION_CCW 2

#define RESET_NONE   (0)
#define RESET_SIGNAL (1 << 0)
#define RESET_RDS    (1 << 1)

#define PI_NONE       -1
#define PI_UNLIKELY    0
#define PI_LIKELY      1
#define PI_VERY_LIKELY 2
#define PI_CORRECT     3

#ifndef M_E
#define M_E 2.71828182845905
#endif

#define ADDR1(a) (((a) & 0xFF0000) >> 16)
#define ADDR2(a) (((a) & 0xFF00) >> 8)
#define ADDR3(a) ((a) & 0xFF)

void setup();
void loop();
inline void handle_rds_interrupt();
inline void handle_rotator();
inline void handle_signal_check();
inline void handle_serial_command();
inline void handle_hw_button();

uint16_t dsp_read_16(uint32_t);
void dsp_write_24(uint32_t, uint32_t);
void dsp_write_16(uint32_t, uint16_t);
void dsp_write_data(const uint8_t*);
void dsp_write_coeff(uint8_t, uint8_t);
bool dsp_set_filter(int8_t);
void dsp_set_deemphasis(uint8_t);
void dsp_read_rds();
float dsp_read_signal(uint8_t);
int8_t dsp_read_multipath(uint8_t);
int8_t dsp_read_usn();

void tune(uint8_t);
bool tune_freq(uint32_t);
void tune_full(uint32_t);
uint32_t get_current_freq();

void scan(bool);

bool set_mode(uint8_t);
void set_agc(uint8_t);
void set_antenna(uint8_t);

void serial_hex(uint8_t);
void serial_signal(float, uint8_t);
void serial_pi(uint16_t, uint8_t);

void signal_reset();
void rds_sync_reset();

void st_pilot();
bool st_pilot_test(uint8_t);

void ir_sendcode(uint32_t);
void ir_carrier(uint16_t);

void setup()
{
    pinMode(RDS_PIN, INPUT);
    pinMode(SDA_PIN, INPUT);
    pinMode(SCL_PIN, INPUT);

    pinMode(POWER_PIN, OUTPUT);
    digitalWrite(POWER_PIN, LOW);

    pinMode(RESET_PIN, OUTPUT);
    digitalWrite(RESET_PIN, LOW);

#if (IR || POWER)
    pinMode(IR_PIN, OUTPUT);
    digitalWrite(IR_PIN, LOW);
#endif

    pinMode(ROT_CW_PIN, OUTPUT);
    digitalWrite(ROT_CW_PIN, LOW);
    pinMode(ROT_CCW_PIN, OUTPUT);
    digitalWrite(ROT_CCW_PIN, LOW);

    pinMode(ANT_A_PIN, OUTPUT);
    digitalWrite(ANT_A_PIN, LOW);
    pinMode(ANT_B_PIN, OUTPUT);
    digitalWrite(ANT_B_PIN, LOW);
    pinMode(ANT_C_PIN, OUTPUT);
    digitalWrite(ANT_C_PIN, LOW);
    pinMode(ANT_D_PIN, OUTPUT);
    digitalWrite(ANT_D_PIN, LOW);
    digitalWrite(ANT[current_ant], HIGH);
    pinMode(BUTTON_PIN, INPUT);
    digitalWrite(BUTTON_PIN, HIGH);

    Serial.begin(SERIAL_PORT_SPEED);
    while(true)
    {
        if(Serial.available() &&
           Serial.read() == 'x')
        {
            while(!Serial.available());
            if(Serial.read() == '\n')
                break;
        }
    }

#if IR_POWER_RESET && (IR || POWER)
    /* Reset the tuner before trying to power it up
       It might be already running! */
    digitalWrite(POWER_PIN, HIGH);
    delay(200);
    digitalWrite(POWER_PIN, LOW);
    delay(2500);
#endif

#if IR
    ir_sendcode(IR_POWER);
    delay(SLEEP_TIME*1000UL);
#elif POWER
    digitalWrite(IR_PIN, HIGH);
    delay(200);
    digitalWrite(IR_PIN, LOW);
    delay(SLEEP_TIME*1000UL);
#endif

    digitalWrite(POWER_PIN, HIGH);
    pinMode(SDA_PIN, OUTPUT);
    pinMode(SCL_PIN, OUTPUT);
    digitalWrite(SDA_PIN, HIGH);
    digitalWrite(SCL_PIN, HIGH);
    delay(100);

#if INIT
    delay(500);
    digitalWrite(RESET_PIN, HIGH);
    delay(100);
    digitalWrite(RESET_PIN, LOW);
    delay(5);
    digitalWrite(RESET_PIN, HIGH);
    delay(100);
    dsp_write_data(DSP_INIT);
#endif

    set_mode(MODE_FM);
    dsp_write_16(DSP_ST_THRESHOLD, ST_THRESHOLD); // 3.75kHz stereo pilot threshold
    dsp_write_16(DSP_VOLUME_SCALER, volume); // set max sound volume
    dsp_set_deemphasis(0); // 50us de-emphasis as default
    tune_freq(current_freq);

    while(Serial.available())
        Serial.read(); /* clear the serial buffer */
    Serial.print("\nOK\n");
}

void loop()
{
    handle_rds_interrupt();
    handle_hw_button();
    handle_rotator();
    handle_signal_check();
    handle_serial_command();
}

inline void handle_rds_interrupt()
{
    if((millis()-last_rds_reset) <= RDS_SYNC_WAIT)
        return;
    if(!digitalRead(RDS_PIN))
        dsp_read_rds();
}

inline void handle_hw_button()
{
    static uint8_t last_state = HIGH;
    static uint8_t state = HIGH;
    static uint32_t timer = 0;
    uint8_t current = digitalRead(BUTTON_PIN);

    if(current != last_state)
        timer = millis();

    if((millis() - timer) > BUTTON_DEBOUNCE &&
       state != current)
    {
        state = current;
        if(state == LOW)
            Serial.print("!\n");
    }
    last_state = current;
}

inline void handle_rotator()
{
    static int8_t state = ROTATION_OFF;
    static int8_t last_state = ROTATION_OFF;
    static int8_t queued = -1;
    static uint32_t timer;

    /* Request rotator stop automatically after a specified time */
    if(state && (millis()-timer) >= ROTATOR_TIMEOUT*1000UL)
        rotator_req = 0;

    /* Nothing to do? */
    if(rotator_req == -1)
        return;

    /* Stop the rotator */
    if(state != ROTATION_OFF)
    {
        digitalWrite(ROT_CW_PIN, LOW);
        digitalWrite(ROT_CCW_PIN, LOW);
        timer = millis();
        last_state = state;
        state = ROTATION_OFF;
    }

    /* Only stop? */
    if(rotator_req == ROTATION_OFF)
        goto rotator_new_state;

    /* Wait before changing a rotation direction */
    if(((last_state == ROTATION_CW && rotator_req == ROTATION_CCW) ||
        (last_state == ROTATION_CCW && rotator_req == ROTATION_CW)) &&
       (millis()-timer) < ROTATOR_DELAY)
    {
        if(queued != rotator_req)
        {
            Serial.print("C-");
            Serial.print(rotator_req, DEC);
            Serial.print('\n');
            queued = rotator_req;
        }
        return; 
    }
    
    if(rotator_req == ROTATION_CW)
        digitalWrite(ROT_CW_PIN, HIGH);
    else
        digitalWrite(ROT_CCW_PIN, HIGH);
    timer = millis();

rotator_new_state:
    Serial.print('C');
    Serial.print(rotator_req, DEC);
    Serial.print('\n');
    state = rotator_req;
    rotator_req = -1;
    queued = -1;
}

inline void handle_signal_check()
{
    static uint32_t timer = 0;
    static uint8_t prev_pos = 0;
    uint32_t current_timer = millis();
    uint8_t curr_pos = (prev_pos+1)%SIGNAL_SAMPLE_COUNT;
    bool current_stereo;
    bool threshold_exceeded;
    float current_level;
    uint16_t interval = (sampling_custom ? sampling_custom : TIMER_INTERVAL/SIGNAL_SAMPLE_COUNT);

    if((current_timer-timer) < interval)
        return;

    /* Always use fast response signal level detector for first few samples. */
    level[curr_pos] = dsp_read_signal((level_fast_countdown || level_fast) ? LEVEL_FAST : LEVEL_SLOW);
    if(level_fast_countdown > 0)
        level_fast_countdown--;

    stereo[curr_pos] = (mode == MODE_FM && dsp_read_16(DSP_ST_19kHz));
    if(sampling_custom)
    {
        /* For custom sampling mode use only current value */
        current_stereo = stereo[curr_pos];
    }
    else
    {
        /* At least 2/3 samples of stereo pilot detector should be positive */
        current_stereo = ((stereo[0] && stereo[1]) ||
                          (stereo[1] && stereo[2]) ||
                          (stereo[0] && stereo[2]));
    }

    /* Mute or unmute audio depending on a squelch (-1 is stereo, otherwise signal) */
    threshold_exceeded = ((squelch_threshold < 0) ? current_stereo : (level[curr_pos] >= squelch_threshold));
    if(threshold_exceeded && !squelch_state)
    {
        dsp_write_16(DSP_VOLUME_SCALER, volume);
        squelch_state = SQUELCH_TIMEOUT;
    }
    else if(squelch_state)
    {
        if(threshold_exceeded)
            squelch_state = SQUELCH_TIMEOUT;
        else
        {
            squelch_state--;
            if(!squelch_state)
                dsp_write_16(DSP_VOLUME_SCALER, 0);
        }
    }

    if(!curr_pos || sampling_custom)
    {
        if(sampling_custom)
            current_level = level[curr_pos];
        else if(level[0] >= 0.0 && level[1] >= 0.0 && level[2] >= 0.0)
            current_level = (level[0] + level[1] + level[2]) / 3.0;
        else
            current_level = ((level[prev_pos] >= 0.0) ? ((level[prev_pos] + level[curr_pos]) / 2.0) : level[curr_pos]);

        Serial.print('S');
        if(!forced_mono)
            Serial.print((current_stereo)?'s':'m');
        else
            Serial.print((current_stereo)?'S':'M');
        serial_signal(current_level, 2);
        Serial.print(',');
        Serial.print(dsp_read_multipath(current_level), DEC);
        Serial.print(',');
        Serial.print(dsp_read_usn(), DEC);
        Serial.print('\n');
    }

    timer = current_timer;
    prev_pos = curr_pos;
}

inline void handle_serial_command()
{
    static char buff[SERIAL_BUFFER_SIZE];
    static uint8_t buff_pos = 0;
    uint8_t n;
    char *ptr;

    if(!Serial.available())
        return;

    buff[buff_pos] = Serial.read();
    if(buff[buff_pos] != '\n')
    {
        /* If this command is too long to
         * fit into a buffer, clip it */
        if(buff_pos != SERIAL_BUFFER_SIZE-1)
            buff_pos++;
        return;
    }

    buff[buff_pos] = '\0';
    buff_pos = 0;

    switch(buff[0])
    {
    case 'x':
        Serial.print("OK\n");
        break;

    case 'T':
        tune_full(atol(buff+1));
        break;

    case 'A':
        n = atol(buff+1);
        if(n < 4)
        {
            set_agc(n);
            Serial.print('A');
            Serial.print(n, DEC);
            Serial.print('\n');
        }
        break;

    case 'V':
        DAA = atol(buff+1) & 0x7F;
        tune(RESET_NONE);
        Serial.print('V');
        Serial.print(DAA, DEC);
        Serial.print('\n');
        break;

    case 'F':
        if(dsp_set_filter(atol(buff+1)))
        {
            current_filter = atol(buff+1);
            Serial.print('F');
            Serial.print(current_filter, DEC);
            Serial.print('\n');
        }
        break;

    case 'D':
        n = atol(buff+1);
        if(n < 3)
        {
            dsp_set_deemphasis(n);
            Serial.print('D');
            Serial.print(n, DEC);
            Serial.print('\n');
        }
        break;

    case 'M':
        n = atol(buff+1);
        if(set_mode(n))
        {
            tune(RESET_SIGNAL | RESET_RDS);
            Serial.print('M');
            Serial.print(n, DEC);
            Serial.print('\n');
        }
        break;

    case 'G':
        Serial.print('G');
        if(buff[1] == '1')
        {
            CONTROL |= B10000000; /* FM RF +6dB gain */
            Serial.print('1');
        }
        else
        {
            CONTROL &= B01111111; /* FM RF standard gain */
            Serial.print('0');
        }
        if(buff[2] == '1')
        {
            CONTROL |= B00010000; /* IF +6dB gain */
            Serial.print('1');
        }
        else
        {
            CONTROL &= B11101111; /* IF standard gain */
            Serial.print('0');
        }
        Serial.print('\n');
        tune(RESET_NONE);
        break;

    case 'S':
        if(buff[1] == 'a')
            scan_start = atol(buff+2);
        else if(buff[1] == 'b')
            scan_end = atol(buff+2);
        else if(buff[1] == 'c')
            scan_step = atol(buff+2);
        else if(buff[1] == 'f')
            scan_filter = atol(buff+2);
        else if(scan_start > 0 && scan_end > 0 && scan_step > 0 && scan_filter >= 0)
            scan((buff[1] == 'm'));
        break;

    case 'Y':
        n = atol(buff+1);
        if(n <= 100)
        {
            volume = (uint16_t)((exp(n/100.0)-1)/(M_E-1) * MAX_VOLUME);
            if(squelch_state)
                dsp_write_16(DSP_VOLUME_SCALER, volume);
            Serial.print('Y');
            Serial.print(n, DEC);
            Serial.print('\n');
        }
        break;

    case 'Q':
        squelch_threshold = atol(buff+1);
        Serial.print('Q');
        Serial.print(squelch_threshold, DEC);
        Serial.print('\n');
        break;

    case 'Z':
        set_antenna(atol(buff+1));
        break;

    case 'C':
        n = atol(buff+1);
        if(n <= ROTATION_CCW)
            rotator_req = n;
        break;

    case 'N':
        st_pilot();
        break;

    case 'B':
        forced_mono = atol(buff+1);
        dsp_write_16(DSP_FORCE_MONO, (forced_mono ? DSP_TRUE : DSP_FALSE));
        Serial.print('B');
        Serial.print(forced_mono ? '1' : '0');
        Serial.print('\n');
        break;

    case 'I':
        sampling_custom = constrain(atol(buff+1), 0, 1000);
        for(ptr = buff+1; *ptr != '\0'; ptr++)
        {
            if(*ptr == ',')
            {
                level_fast = (*(ptr+1) == '1');
                break;
            }
        }
        Serial.print('I');
        Serial.print(sampling_custom, DEC);
        Serial.print(',');
        Serial.print(level_fast, DEC);
        Serial.print('\n');
        break;

    case 'X':
        TWCR = 0; /* release SDA and SCL lines used by hardware I2C */
        digitalWrite(POWER_PIN, LOW);
        Serial.print("X\n");
        delay(10);
        asm("jmp 0");
        break;
    }
}

uint16_t dsp_read_16(uint32_t addr)
{
    uint16_t buffer;
    i2c.start(DSP_I2C | I2C_WRITE);
    i2c.write(ADDR1(addr));
    i2c.write(ADDR2(addr));
    i2c.write(ADDR3(addr));
    i2c.restart(DSP_I2C | I2C_READ);
    buffer = ((uint16_t)i2c.read(false) << 8);
    buffer |= i2c.read(true);
    i2c.stop();
    return buffer;
}

void dsp_write_24(uint32_t addr, uint32_t data)
{
    i2c.start(DSP_I2C | I2C_WRITE);
    i2c.write(ADDR1(addr));
    i2c.write(ADDR2(addr));
    i2c.write(ADDR3(addr));
    i2c.write((uint8_t)(data >> 16));
    i2c.write((uint8_t)(data >> 8));
    i2c.write((uint8_t)data);
    i2c.stop();
}

void dsp_write_16(uint32_t addr, uint16_t data)
{
    i2c.start(DSP_I2C | I2C_WRITE);
    i2c.write(ADDR1(addr));
    i2c.write(ADDR2(addr));
    i2c.write(ADDR3(addr));
    i2c.write((uint8_t)(data >> 8));
    i2c.write((uint8_t)data);
    i2c.stop();
}

void dsp_write_data(const uint8_t* data)
{
    uint16_t i = 1;
    uint8_t len = pgm_read_byte_near(data), pos;
    while(len)
    {
        i2c.start(DSP_I2C | I2C_WRITE);
        for(pos=0; pos<len; pos++)
            i2c.write(pgm_read_byte_near(data+i+pos));
        i2c.stop();
        i += pos;
        len = pgm_read_byte_near(data+(i++));
    }
}

void dsp_write_coeff(uint8_t bank, uint8_t filter)
{
    uint8_t i = 0;
    uint16_t address = 0x0C00 + 32 * bank;
    while(i<64)
    {
        i2c.start(DSP_I2C | I2C_WRITE);
        i2c.write(0x01);
        i2c.write(address >> 8);
        i2c.write(address & 0xFF);
        i2c.write(pgm_read_byte_near(filters[filter]+(i++)));
        i2c.write(pgm_read_byte_near(filters[filter]+(i++)));
        i2c.write(0x00);
        i2c.stop();
        address++;
    }
}

bool dsp_set_filter(int8_t f)
{
    static uint8_t current_filter_flag = 0;
    uint8_t i;

    if(f >= 0 && f < filters_count)
    {
        /* fixed filter bandwidth */
        if(mode == MODE_AM) /* workaround for AM (?) */
        {
            for(i=0; i<16; i++)
                dsp_write_coeff(i, f);
            return true;
        }

        /* use another filter bank to avoid audio 'popping'
           when changing bandwidth next to stronger adjacent station */
        current_filter_flag = (current_filter_flag?0:1);

        /* write the FIR filter coefficients into $15 or $14 filter bank */
        dsp_write_coeff(0x0F - current_filter_flag, f);
        dsp_write_24(TDSP1_X_CIBW_1_FirCtlFix, 0x00000F-current_filter_flag); /* $15 or $14 filter */
        dsp_write_24(TDSP1_X_CIBW_4_FirCtlFix, 0x00000F-current_filter_flag); /* $15 or $14 filter */
        dsp_write_24(TDSP1_X_CIBW_1_pFirCtl, (uint16_t)TDSP1_X_CIBW_1_FirCtlFix); /* relative address */
        dsp_write_24(TDSP1_X_CIBW_4_pFirCtl, (uint16_t)TDSP1_X_CIBW_4_FirCtlFix); /* relative address */
        return true;
    }
    else if(mode == MODE_FM && f == -1)
    {
        /* adaptive filter bandwidth */
        dsp_write_24(TDSP1_X_CIBW_1_pFirCtl, (uint16_t)TDSP1_X_CIBW_1_FirCtl); /* relative address */
        dsp_write_24(TDSP1_X_CIBW_4_pFirCtl, (uint16_t)TDSP1_X_CIBW_4_FirCtl); /* relative address */
        for(i=0; i<16; i++)
            dsp_write_coeff(i, adaptive_filters_set[i]);
        return true;
    }
    return false;
}

void dsp_set_deemphasis(uint8_t d)
{
    i2c.start(DSP_I2C | I2C_WRITE);
    i2c.write(ADDR1(DSP_DEEMPHASIS));
    i2c.write(ADDR2(DSP_DEEMPHASIS));
    i2c.write(ADDR3(DSP_DEEMPHASIS));
    switch(d)
    {
    default:
    case 0: /* 50us */
        i2c.write(0x02);
        i2c.write(0xC0);
        i2c.write(0x04);
        i2c.write(0xE4);
        i2c.write(0x00);
        i2c.write(0x85);
        break;
    case 1: /* 75us */
        i2c.write(0x01);
        i2c.write(0xF6);
        i2c.write(0x05);
        i2c.write(0xC3);
        i2c.write(0x00);
        i2c.write(0x85);
        break;
    case 2: /* off */
        i2c.write(0x07);
        i2c.write(0xFF);
        i2c.write(0x00);
        i2c.write(0x00);
        i2c.write(0x00);
        i2c.write(0x00);
        break;
    }
    i2c.stop();
}

void dsp_read_rds()
{
    static uint16_t pi_buffer[PI_BUFFER_SIZE];
    static uint8_t pi_buffer_errorfree[PI_BUFFER_SIZE/8];
    static uint16_t pi_val;
    static uint32_t rds_timer = 0;
    static uint8_t rds_buffer[4];
    static uint8_t rds_status_buffer;
    uint8_t status = dsp_read_16(DSP_RDS_STATUS);
    uint16_t buffer = dsp_read_16(DSP_RDS_DATA);
    uint8_t current_pi_count = 0;
    uint8_t current_pi_errorfree = 0;
    int8_t current_pi_state;
    uint8_t i;

    switch(status & B11111100)
    {
    case 0x00: /* fast PI mode block */
    case 0x80: /* block A */
        pi_buffer[pi_pos] = buffer;
        if(status == 0x80)
            pi_buffer_errorfree[pi_pos/8] |= (1 << (pi_pos%8));
        else
            pi_buffer_errorfree[pi_pos/8] &= ~(1 << (pi_pos%8));
        pi_pos = (pi_pos+1)%PI_BUFFER_SIZE;

        if(pi_buffer_fill < PI_BUFFER_SIZE)
            pi_buffer_fill++;

        for(i=0; i<pi_buffer_fill; i++)
        {
            if(pi_buffer[i] == buffer)
            {
                current_pi_count++;
                current_pi_errorfree += (pi_buffer_errorfree[i/8] & (1 << (i%8)) ? 1 : 0);
            }
        }

        if(current_pi_count >= 2 && current_pi_errorfree >= 2)
            current_pi_state = PI_CORRECT;
        else if(current_pi_count >= 2 && current_pi_errorfree == 1)
            current_pi_state = PI_VERY_LIKELY;
        else if(current_pi_count >= 3 && !current_pi_errorfree)
            current_pi_state = PI_LIKELY;
        else if((current_pi_errorfree && current_pi_count == 1) ||
                (!current_pi_errorfree && current_pi_count == 2) ||
                (pi_state == PI_CORRECT && pi_val == buffer))
            current_pi_state = PI_UNLIKELY;
        else
            break;

        serial_pi(buffer, current_pi_state);
        if(pi_state <= current_pi_state)
        {
            pi_state = current_pi_state;
            pi_val = buffer;
        }
        break;
    case 0x84: /* block B */
        /* wait for block D before sending anything to the serial */
        rds_buffer[0] = buffer >> 8;
        rds_buffer[1] = (uint8_t)buffer;
        rds_status_buffer = status&B11;
        rds_status_buffer |= B111100;
        rds_timer = millis();
        break;
    case 0x88: /* block C */
    case 0x90: /* block C' */
        rds_buffer[2] = buffer >> 8;
        rds_buffer[3] = (uint8_t)buffer;
        rds_status_buffer &= B0011;
        rds_status_buffer |= (status&B11) << 2;
        break;
    case 0x8C: /* block D */
        /* Is this block related to the block B from buffer? */
        if((millis()-rds_timer) > 50)
            break;

        /* Is there any valid PI code in the buffer? */
        if(pi_state < PI_LIKELY)
            break;
        for(i=0; i<pi_buffer_fill; i++)
        {
            if(pi_buffer[i] == pi_val)
            {
                rds_status_buffer &= B001111;
                rds_status_buffer |= (status&B11) << 4;
                Serial.print('R');
                serial_hex(rds_buffer[0]);
                serial_hex(rds_buffer[1]);
                serial_hex(rds_buffer[2]);
                serial_hex(rds_buffer[3]);
                serial_hex(buffer >> 8);
                serial_hex(buffer);
                serial_hex(rds_status_buffer);
                Serial.print('\n');
                break;
            }
        }
        break;
    }
}

float dsp_read_signal(uint8_t type)
{
    float buffer;
    i2c.start(DSP_I2C | I2C_WRITE);
    if(mode == MODE_FM && type == LEVEL_FAST)
    {
        i2c.write(ADDR1(DSP_FM_LEVEL_FAST));
        i2c.write(ADDR2(DSP_FM_LEVEL_FAST));
        i2c.write(ADDR3(DSP_FM_LEVEL_FAST));
    }
    else if(mode == MODE_FM)
    {
        i2c.write(ADDR1(DSP_FM_LEVEL));
        i2c.write(ADDR2(DSP_FM_LEVEL));
        i2c.write(ADDR3(DSP_FM_LEVEL));
    }
    else
    {
        i2c.write(ADDR1(DSP_AM_LEVEL));
        i2c.write(ADDR2(DSP_AM_LEVEL));
        i2c.write(ADDR3(DSP_AM_LEVEL));
    }
    i2c.restart(DSP_I2C | I2C_READ);
    buffer = i2c.read(false);
    buffer += (uint16_t)((i2c.read(false) << 8) | i2c.read(true)) / 65536.0;
    i2c.stop();

    if(mode == MODE_FM)
        return (buffer * 0.797 + 3.5);

    return buffer;
}

int8_t dsp_read_multipath(uint8_t level)
{
    uint32_t multipath_min, multipath_max, output;
    if(mode != MODE_FM)
        return -1;

    /*  Unfortunately, the multipath detector slighly changes
     *  its output depending on a current signal level. To
     *  overcome this and provide a relative output that
     *  always starts at 0%, a linear correction of usable
     *  detector segment range has been estimated.
     *  For 83dBf signal, the detector range is ~13-51% FS
     *  For 29dBf signal, the detector range is ~ 8-46% FS
     *  Full scale is 0x000000-0x7FFFFF.
     */
    multipath_min = 7723 * (uint32_t)level + 450000;
    multipath_max = multipath_min + 3187670; /* 38% of FS */

    i2c.start(DSP_I2C | I2C_WRITE);
    i2c.write(ADDR1(DSP_FM_MULTIPATH));
    i2c.write(ADDR2(DSP_FM_MULTIPATH));
    i2c.write(ADDR3(DSP_FM_MULTIPATH));
    i2c.restart(DSP_I2C | I2C_READ);
    output = (uint32_t)i2c.read(false) << 16;
    output |= (uint16_t)i2c.read(true) << 8;
    output |= i2c.read(true);
    i2c.stop();
    output = constrain(output, multipath_min, multipath_max);
    output = map(output, multipath_min, multipath_max, 0, 100);
    return output;
}

int8_t dsp_read_usn()
{
    static int32_t last_value = -1;
    uint32_t usn_min, usn_max, output, tmp;
    if(mode != MODE_FM || current_filter != -1)
    {
        last_value = -1;
        return -1;
    }

    i2c.start(DSP_I2C | I2C_WRITE);
    i2c.write(ADDR1(DSP_ULTRASONIC_NOISE));
    i2c.write(ADDR2(DSP_ULTRASONIC_NOISE));
    i2c.write(ADDR3(DSP_ULTRASONIC_NOISE));
    i2c.restart(DSP_I2C | I2C_READ);
    output = (uint32_t)i2c.read(false) << 16;
    output |= (uint16_t)i2c.read(true) << 8;
    output |= i2c.read(true);
    i2c.stop();

    if(last_value != -1)
    {
        output += last_value;
        output /= 2;
    }

    last_value = output;
    usn_min = 251658;  /* 0.03 of FS */
    usn_max = 3355442; /* 0.40 of FS */
    output = constrain(output, usn_min, usn_max);
    output = map(output, usn_min, usn_max, 0, 100);
    return output;
}

void tune(uint8_t reset_flags)
{
    i2c.start(DSP_I2C | I2C_WRITE);
    i2c.write(0x00);
    i2c.write(0xFF);
    i2c.write(0xFF);
    i2c.restart(IF_I2C | I2C_WRITE);
    i2c.write(0x80);
    i2c.write(CONTROL);
    i2c.write(PLL>>8);
    i2c.write(PLL&0xFF);
    i2c.write(DAA);
    i2c.write(AGC);
    i2c.write(BAND);
    i2c.stop();
    delay(4);

    if(!scan_flag)
    {
        if(reset_flags & RESET_RDS && mode == MODE_FM)
            rds_sync_reset();
        if(reset_flags & RESET_SIGNAL)
            signal_reset();
    }
}

bool tune_freq(uint32_t freq) // ***Modified by F4CMB***
{
    int32_t diff;
    uint8_t flags = 0;

    if ((freq>=55000) && (freq<=137000)) // FM BAND (extended)
    {
        if(freq % 50 || freq>108000 || scan_flag)
        {
            PLL = ((freq+10700)*2)/10;
            BAND = B00110001; /* 5kHz step, fref=10kHz */
        }
        else
        {
            /* use 50kHz step if possible, because
               in other cases the audio output isn't clear */
            PLL = ((freq+10700)*2)/100;
            BAND = B00100001; /* 50kHz step, fref=100kHz */
        }
    }
    else if ((freq>=100) && (freq<=1900)) // LW & MW BAND
    {
        PLL = ((freq+10700)*20)/20;
        BAND = B11101101;
    }
    else if ((freq>=1901) && (freq<=5899)) // AM SW 120m to 60m
    {
        PLL = ((freq+10700)*16)/10;
        BAND = B11010001;
    }
    else if ((freq>=5900) && (freq<=13870)) // AM SW 49m to 22m
    {
        PLL = ((freq+10700)*10)/10;
        BAND = B10110001;
    }
    else if ((freq>=13871) && (freq<=19020)) // AM SW 25m to 15m
    {
        PLL = ((freq+10700)*8)/10;
        BAND = B10010001;
    }
    else if ((freq>=19021) && (freq<=30000)) // AM SW 16m to 11m
    {
        PLL = ((freq+10700)*6)/10;
        BAND = B01110001;
    }
    else
    {
        return false;
    }

    /* Reset buffers when tuning to the same frequency or more
       than BUFF_RESET_FREQ_OFFSET kHz from previous frequency */
    diff = current_freq-freq;
    if(diff == 0 || diff >= BUFF_RESET_FREQ_OFFSET || -diff >= BUFF_RESET_FREQ_OFFSET)
        flags |= (RESET_RDS | RESET_SIGNAL);

    align(freq);
    tune(flags);
    return true;
}

void tune_full(uint32_t new_freq)
{
    if(tune_freq(new_freq))
    {
        Serial.print('V');
        Serial.print(DAA&0x7F, DEC);
        Serial.print("\nT");
        current_freq = get_current_freq();
        Serial.print(current_freq, DEC);
        Serial.print('\n');
    }
}

uint32_t get_current_freq()
{
    if(BAND == B00100001)
        return 100*(uint32_t)PLL/2-10700;
    else if(BAND == B00110001)
        return 10*(uint32_t)PLL/2-10700;
    else if(BAND == B11101101)
        return 20*(uint32_t)PLL/20-10700;
    else if(BAND == B11010001)
        return 10*(uint32_t)PLL/16-10700;
    else if(BAND == B10110001)
        return 10*(uint32_t)PLL/10-10700;
    else if(BAND == B10010001)
        return 10*(uint32_t)PLL/8-10700;
    else if(BAND == B01110001)
        return 10*(uint32_t)PLL/6-10700;
    else
        return 0;
}

void scan(bool continous)
{
    /* save current IF settings */
    uint8_t _CONTROL = CONTROL;
    uint16_t _PLL = PLL;
    uint8_t _DAA = DAA;
    uint8_t _AGC = AGC;
    uint8_t _BAND = BAND;
    uint32_t freq;

    if(squelch_state) /* mute audio during scan */
        dsp_write_16(DSP_VOLUME_SCALER, 0);

    scan_flag = true;
    dsp_set_filter(scan_filter);
    tune_freq(scan_start);
    do
    {
        Serial.print('U');
        for(freq = scan_start; freq <= scan_end; freq += scan_step)
        {
            tune_freq(freq);
            if(freq == scan_start)
                delay(4);
            Serial.print(get_current_freq(), DEC);
            Serial.print('=');
            serial_signal(dsp_read_signal(LEVEL_FAST), 1);
            Serial.print(',');
        }
        Serial.print('\n');
    }
    while(continous && !Serial.available());
    scan_flag = false;

    /* restore saved IF settings */
    dsp_set_filter(current_filter);
    CONTROL = _CONTROL;
    PLL = _PLL;
    DAA = _DAA;
    AGC = _AGC;
    BAND = _BAND;
    tune(RESET_SIGNAL | RESET_RDS);

    if(squelch_state) /* unmute */
        dsp_write_16(DSP_VOLUME_SCALER, volume);
}

bool set_mode(uint8_t new_mode)
{
    if(mode != MODE_FM && mode != MODE_AM)
        return false;
    
    mode = new_mode;
    if(mode == MODE_FM)
    {
        dsp_write_data(DSP_FM);
        current_filter = -1;
        dsp_set_filter(current_filter);
    }
    else
    {
        dsp_write_data(DSP_AM);
    }
    return true;
}

void set_agc(uint8_t n)
{
    switch(n)
    {
    case 0: /* highest */
        AGC &= B11110011;
        break;
    case 1: /* high */
        AGC &= B11110111;
        AGC |= B00000100;
        break;
    default:
    case 2: /* medium */
        AGC &= B11111011;
        AGC |= B00001000;
        break;
    case 3: /* low */
        AGC |= B00001100;
        break;
    }
    tune(RESET_NONE);
}

void set_antenna(uint8_t n)
{
    if(n < ANT_n)
    {
        digitalWrite(ANT[current_ant], LOW);
        digitalWrite(ANT[n], HIGH);
        current_ant = n;
        signal_reset();
        delay(ANTENNA_SWITCH_DELAY);
        rds_sync_reset();
        Serial.print('Z');
        Serial.print(n, DEC);
        Serial.print('\n');
    }
}

void serial_hex(uint8_t val)
{
    Serial.print((val >> 4) & 0xF, HEX);
    Serial.print(val & 0xF, HEX);
}

void serial_signal(float level, uint8_t precision)
{
    uint8_t n = (level-(int)level)*pow(10, precision);

    Serial.print((int)level, DEC);
    Serial.write('.');
    if(precision == 2 && n < 10)
        Serial.write('0');
    Serial.print(n, DEC);
}

void serial_pi(uint16_t pi, uint8_t err)
{
    Serial.print('P');
    serial_hex(pi >> 8);
    serial_hex(pi & 0xFF);
    err = PI_CORRECT - err;
    while(err--)
        Serial.print('?');
    Serial.print('\n');
}

void signal_reset()
{
    uint8_t i;
    for(i=0; i<SIGNAL_SAMPLE_COUNT; i++)
    {
        level[i] = -1.0;
        stereo[i] = false;
    }
    level_fast_countdown = 250 / (sampling_custom ? sampling_custom : (TIMER_INTERVAL/SIGNAL_SAMPLE_COUNT));
    if(squelch_state > SIGNAL_SAMPLE_COUNT)
        squelch_state = SIGNAL_SAMPLE_COUNT;
}

void rds_sync_reset()
{
    dsp_write_16(DSP_RDS_CONTROL, 0x0060); // fast pi mode
    pi_state = PI_NONE;
    pi_buffer_fill = 0;
    pi_pos = 0;
    last_rds_reset = millis();
}

void st_pilot()
{
    uint8_t i, j, level = 0;
    // if the subcarrier is present (>3kHz), try to guess the level
    if(st_pilot_test(30))
    {
        // check from 15kHz to 3kHz in 1kHz step
        for(i=150; i>30; i-=10)
        {
            if(st_pilot_test(i))
            {
                level = i;
                // the stereo subcarrier is found
                // find the exact value
                for(j=i+10; j>i; j--)
                {
                    if(st_pilot_test(j))
                    {
                        level = j;
                        break;
                    }
                }
                break;
            }
        }
    }
    dsp_write_16(DSP_ST_THRESHOLD, ST_THRESHOLD);
    Serial.write('N');
    Serial.print(level, DEC);
    Serial.write('\n');
}

bool st_pilot_test(uint8_t level)
{
    dsp_write_16(DSP_ST_THRESHOLD, (uint16_t)((1.08 * ((level-1)/10.0)) / 100.0 * 2048));
    delay(2);
    return dsp_read_16(DSP_ST_19kHz);
}

/* IR support based on Olivier F4CMB's code */
void ir_sendcode(uint32_t code)
{
    int8_t n, i;
    for(n=0; n<3; n++)
    {
        cli();
        ir_carrier(2400);
        delayMicroseconds(600);
        for(i=19; i>=0; i--)
        {
            ir_carrier(((code >> i) & 1) ? 1200 : 600);
            delayMicroseconds(600);
        }
        sei();
        delay(75);
    }
}

void ir_carrier(uint16_t time)
{
    uint8_t i;
    for(i=0; i<(time/25); i++) /* approx. 40kHz */
    {
#if (IR_PIN >= 0 && IR_PIN <= 7)
        PORTD |= (1<<IR_PIN);
#elif (IR_PIN >= 8 && IR_PIN <= 13)
        PORTB |= (1<<(IR_PIN-8));
#endif
        delayMicroseconds(12);
#if (IR_PIN >= 0 && IR_PIN <= 7)
        PORTD &= ~(1<<IR_PIN);
#elif (IR_PIN >= 8 && IR_PIN <= 13)
        PORTB &= ~(1<<(IR_PIN-8));
#endif
        delayMicroseconds(12);
    }
}
