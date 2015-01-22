/*
 *  XDR-I2C 2015-01-22
 *  Copyright (C) 2012-2015  Konrad Kosmatka
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

/* Maximum audio output level (0~2047) */
#define MAX_VOLUME 2047

/* Send DSP initialization data on start (for tuners without stock controller) */
#define INIT 0

/* Pinout */
#define RDS_PIN     2
#define IR_PIN      3
#define POWER_PIN   4
#define RESET_PIN   5
#define ROT_CW_PIN  6
#define ROT_CCW_PIN 7
#define ANT_A_PIN   8
#define ANT_B_PIN   9
#define ANT_C_PIN  10
#define ANT_D_PIN  11
#define SDA_PIN    A4
#define SCL_PIN    A5

#define MODE_FM 0
#define MODE_AM 1
#define ROTATION_OFF 0
#define ROTATION_CW  1
#define ROTATION_CCW 2

#ifndef M_E
#define M_E 2.71828182845905
#endif

// Serial
#define SERIAL_BUFFER_SIZE 16
char buff[SERIAL_BUFFER_SIZE];
uint8_t buff_pos = 0;

// TEF6730 IF
uint8_t CONTROL = 0x00;
uint16_t PLL;
uint8_t DAA = 0x00;
uint8_t AGC = 0xC8;
uint8_t BAND;

// RDS
#define PI_BUFFER_SIZE 64
uint32_t rds_timer = 0;
uint16_t pi_buffer[PI_BUFFER_SIZE];
uint8_t rds_buffer[4];
uint8_t rds_status_buffer;
uint8_t pi_pos = 0;
bool pi_checked = false;

// Scan
bool scan_flag = false;
uint32_t scan_start = 0;
uint32_t scan_end = 0;
uint8_t scan_step = 0;
uint8_t scan_filter = 0;

// Antenna switch
const uint8_t ANT[] = {ANT_A_PIN, ANT_B_PIN, ANT_C_PIN, ANT_D_PIN};
const uint8_t ANT_n = sizeof(ANT)/sizeof(uint8_t);
uint8_t current_ant = 0;

// Signal level & squelch
#define TIMER_INTERVAL   33
#define SQUELCH_TIMEOUT   6
uint32_t timer = 0;
float prev_level = 0.0;
bool prev_stereo = false;
bool print_level = false;
int8_t squelch_threshold = 0;
uint8_t squelch_state = 0;

// Other
#define ST_THRESHOLD 0x052
uint8_t mode; // FM/AM demod
int8_t current_filter = -1; // current FIR filter (-1 is adaptive)
uint8_t current_filter_flag = 0;
uint16_t volume = MAX_VOLUME; // audio volume control
uint32_t rotator_timer = 0;

TwiMaster i2c(false);

uint16_t dsp_read_16(uint32_t);
void dsp_write_24(uint32_t, uint32_t);
void dsp_write_16(uint32_t, uint16_t);
void dsp_write_data(const uint8_t*);
void dsp_write_coeff(uint8_t, uint8_t);
void dsp_set_filter(int8_t);
void dsp_set_deemphasis(uint8_t);
void dsp_read_rds();
void tune(boolean);
bool tune_freq(uint32_t);
void mode_FM();
void mode_AM();
void scan(bool);
uint32_t get_current_freq();
void ant_switch(uint8_t);
float signal_level();
void serial_signal(float);
void serial_hex(uint8_t);
void serial_write_signal(float, uint8_t);
void signal_reset();
void rds_sync_reset();
void st_pilot();
bool st_pilot_test(uint8_t level);

void sendcode(uint32_t);
void carrier(int);
void start();
void one();
void zero();

#define ADDR1(a) ((a & 0xFF0000) >> 16)
#define ADDR2(a) ((a & 0xFF00) >> 8)
#define ADDR3(a) (a & 0xFF)

void setup(void)
{
    pinMode(RDS_PIN, INPUT);
    pinMode(SDA_PIN, INPUT);
    pinMode(SCL_PIN, INPUT);
    pinMode(POWER_PIN, OUTPUT);
    digitalWrite(POWER_PIN, LOW);
    pinMode(RESET_PIN, OUTPUT);
    digitalWrite(RESET_PIN, LOW);
    pinMode(IR_PIN, OUTPUT);
    digitalWrite(IR_PIN, LOW);
    pinMode(ROT_CW_PIN, OUTPUT);
    pinMode(ROT_CCW_PIN, OUTPUT);
    digitalWrite(ROT_CW_PIN, LOW);
    digitalWrite(ROT_CCW_PIN, LOW);
    pinMode(ANT_D_PIN, OUTPUT);
    pinMode(ANT_A_PIN, OUTPUT);
    pinMode(ANT_B_PIN, OUTPUT);
    pinMode(ANT_C_PIN, OUTPUT);
    pinMode(ANT_D_PIN, OUTPUT);
    digitalWrite(ANT[0], LOW);
    digitalWrite(ANT[1], LOW);
    digitalWrite(ANT[2], LOW);
    digitalWrite(ANT[3], LOW);
    digitalWrite(ANT[current_ant], HIGH);
    Serial.begin(115200);

    while(true)
    {
        if(Serial.available() > 0)
        {
            if(Serial.read() == 'x')
            {
                while(!Serial.available());
                if(Serial.read() == '\n')
                    break;
            }
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
    for(uint8_t i=0; i<10; i++)
    {
        sendcode(0xA8BC8);
        delayMicroseconds(10000);
    }
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

    mode_FM(); // switch mode to FM and use adaptive filter bandwidth
    dsp_write_16(DSP_VOLUME_SCALER, volume); // set max sound volume
    dsp_set_deemphasis(0); // 50us de-emphasis as default
    tune_freq(87500); // tune to 87.500 MHz
    dsp_write_16(DSP_ST_THRESHOLD, ST_THRESHOLD); // 3.75kHz stereo pilot threshold

    while(Serial.available() > 0)
    {
        Serial.read(); // clear the serial buffer
    }
    Serial.print("\nOK\n");
}

void loop()
{
    uint8_t i;

    if(!digitalRead(RDS_PIN))
    {
        dsp_read_rds();
    }

    if(rotator_timer && (millis()-rotator_timer) >= ROTATOR_TIMEOUT*1000UL)
    {
        digitalWrite(ROT_CW_PIN, LOW);
        digitalWrite(ROT_CCW_PIN, LOW);
        rotator_timer = 0;
        Serial.write("C0\n");
    }

    // check signal level and 19kHz subcarrier every TIMER_INTERVAL,
    // mute or unmute audio depending on a squelch state
    // and display the signal level every TIMER_INTERVAL*2
    if((millis()-timer) >= TIMER_INTERVAL)
    {
        float level = signal_level();
        bool stereo = (mode == MODE_FM && dsp_read_16(DSP_ST_19kHz));
        bool threshold_exceeded = (squelch_threshold < 0 && stereo) || (squelch_threshold >= 0 && level > squelch_threshold);
        timer = millis();

        if(threshold_exceeded && !squelch_state)
        {
            dsp_write_16(DSP_VOLUME_SCALER, volume);
            squelch_state = SQUELCH_TIMEOUT;
        }
        else if(squelch_state)
        {
            if(threshold_exceeded)
            {
                squelch_state = SQUELCH_TIMEOUT;
            }
            else
            {
                squelch_state--;
                if(!squelch_state)
                {
                    dsp_write_16(DSP_VOLUME_SCALER, 0);
                }
            }
        }

        if(print_level)
        {
            Serial.print('S');
            Serial.print((prev_stereo||stereo)?'s':'m');
            serial_write_signal(((prev_level > 0) ? ((prev_level + level) / 2.0) : level), 2);
            Serial.print('\n');
        }
        else
        {
            prev_level = level;
            prev_stereo = stereo;
        }

        print_level = !print_level;
    }

    if(Serial.available() > 0)
    {
        buff[buff_pos] = Serial.read();
        if(buff[buff_pos] != '\n' && buff_pos != SERIAL_BUFFER_SIZE-1)
        {
            buff_pos++;
        }
        else
        {
            buff[buff_pos] = 0x00;
            buff_pos = 0;
            switch (buff[0])
            {
            case 'x':
                Serial.print("OK\n");
                break;

            case 'T': // frequency change
                if(tune_freq(atol(buff+1)))
                {
                    Serial.print('V');
                    Serial.print(DAA&0x7F, DEC);
                    Serial.print("\nT");
                    Serial.print(get_current_freq(), DEC);
                    Serial.print('\n');
                }
                break;

            case 'A': // RF AGC threshold
                switch (atol(buff+1))
                {
                case 0:
                    AGC &= B11110011; // highest
                    break;
                case 1:
                    AGC &= B11110111; // high
                    AGC |= B00000100;
                    break;
                case 2:
                    AGC &= B11111011; // medium
                    AGC |= B00001000;
                    break;
                case 3:
                    AGC |= B00001100; // low
                    break;
                default:
                    return;
                }
                tune(false);
                break;

            case 'V': // set 1st antenna circuit tuning voltage
                DAA = atol(buff+1) & 0x7F;
                tune(false);
                break;

            case 'F': // change FIR filters
                current_filter = atol(buff+1);
                dsp_set_filter(current_filter);
                break;

            case 'D': // change the de-emphasis
                dsp_set_deemphasis(atol(buff+1));
                break;

            case 'M': // change the mode (added by F4CMB)
                switch(atol(buff+1))
                {
                case MODE_FM:
                    mode_FM();
                    break;
                case MODE_AM:
                    mode_AM();
                    break;
                }
                tune(false);
                break;

            case 'G':
                if(buff[1] == '1')
                {
                    CONTROL |= B10000000; // FM RF +6dB gain
                }
                else
                {
                    CONTROL &= B01111111; // FM RF standard gain
                }

                if(buff[2] == '1')
                {
                    CONTROL |= B00010000; // IF +6dB gain
                }
                else
                {
                    CONTROL &= B11101111; // IF standard gain
                }
                tune(false);
                break;

            case 'S':
                if(buff[1] == 'a')
                {
                    scan_start = atol(buff+2);
                }
                else if(buff[1] == 'b')
                {
                    scan_end = atol(buff+2);
                }
                else if(buff[1] == 'c')
                {
                    scan_step = atol(buff+2);
                }
                else if(buff[1] == 'f')
                {
                    scan_filter = atol(buff+2);
                }
                else if(scan_start > 0 && scan_end > 0 && scan_step > 0  && scan_filter >= 0)
                {
                    if(buff[1] == 'm')
                    {
                        scan(true); // multiple (continous) scan
                    }
                    else
                    {
                        scan(false); // single scan
                    }
                }
                break;

            case 'Y': // audio volume scaler
                volume = (uint16_t)((exp(atol(buff+1)/100.0)-1)/(M_E-1) * MAX_VOLUME);
                if(squelch_state)
                {
                    dsp_write_16(DSP_VOLUME_SCALER, volume);
                }
                break;

            case 'Z': // antenna switch
                ant_switch(atol(buff+1));
                break;

            case 'C': // antenna rotation
                switch(atol(buff+1))
                {
                case ROTATION_OFF:
                    digitalWrite(ROT_CW_PIN, LOW);
                    digitalWrite(ROT_CCW_PIN, LOW);
                    rotator_timer = 0;
                    break;

                case ROTATION_CW:
                    digitalWrite(ROT_CW_PIN, HIGH);
                    digitalWrite(ROT_CCW_PIN, LOW);
                    rotator_timer = millis();
                    break;

                case ROTATION_CCW:
                    digitalWrite(ROT_CW_PIN, LOW);
                    digitalWrite(ROT_CCW_PIN, HIGH);
                    rotator_timer = millis();
                    break;
                }
                break;

            case 'Q': // squelch
                squelch_threshold = atol(buff+1);
                break;
                
            case 'N':
                st_pilot();
                break;

            case 'X': // shutdown
                TWCR = 0; // release SDA and SCL lines used by hardware I2C
                digitalWrite(POWER_PIN, LOW);
                Serial.print("X\n");
                delay(10);
                asm("jmp 0");
                break;
            }
        }
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
    while(len != 0x00)
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

void dsp_set_filter(int8_t f)
{
    if(f >= 0) // fixed filter bandwidth
    {
        if(mode == MODE_AM)
        {
            // workaround for AM (?)
            for(uint8_t i=0; i<16; i++)
                dsp_write_coeff(i, f);
            return;
        }

        // use another filter bank to avoid audio 'popping'
        // when changing bandwidth next to stronger adjacent station
        current_filter_flag = (current_filter_flag?0:1);

        // write the FIR filter coefficients into $15 or $14 filter bank
        dsp_write_coeff(0x0F - current_filter_flag, f);

        dsp_write_24(TDSP1_X_CIBW_1_FirCtlFix, 0x00000F-current_filter_flag); // $15 or $14 filter
        dsp_write_24(TDSP1_X_CIBW_4_FirCtlFix, 0x00000F-current_filter_flag); // $15 or $14 filter
        dsp_write_24(TDSP1_X_CIBW_1_pFirCtl, (uint16_t)TDSP1_X_CIBW_1_FirCtlFix); // relative address
        dsp_write_24(TDSP1_X_CIBW_4_pFirCtl, (uint16_t)TDSP1_X_CIBW_4_FirCtlFix); // relative address
    }
    else if(mode == MODE_FM) // adaptive filter bandwidth
    {
        dsp_write_24(TDSP1_X_CIBW_1_pFirCtl, (uint16_t)TDSP1_X_CIBW_1_FirCtl); // relative address
        dsp_write_24(TDSP1_X_CIBW_4_pFirCtl, (uint16_t)TDSP1_X_CIBW_4_FirCtl); // relative address

        for(uint8_t i=0; i<16; i++)
            dsp_write_coeff(i, adaptive_filters_set[i]);
    }
}

void dsp_set_deemphasis(uint8_t d)
{
    if(d >= 3)
    {
        return;
    }

    i2c.start(DSP_I2C | I2C_WRITE);
    i2c.write(ADDR1(DSP_DEEMPHASIS));
    i2c.write(ADDR2(DSP_DEEMPHASIS));
    i2c.write(ADDR3(DSP_DEEMPHASIS));
    switch (d)
    {
    case 0: // 50us
        i2c.write(0x02);
        i2c.write(0xC0);
        i2c.write(0x04);
        i2c.write(0xE4);
        i2c.write(0x00);
        i2c.write(0x85);
        break;
    case 1: // 75us
        i2c.write(0x01);
        i2c.write(0xF6);
        i2c.write(0x05);
        i2c.write(0xC3);
        i2c.write(0x00);
        i2c.write(0x85);
        break;
    case 2: // off
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
    uint8_t status, current_pi_count = 0;
    uint16_t buffer;

    status = dsp_read_16(0x000030);
    buffer = dsp_read_16(0x000031);

    switch(status & B11111100)
    {
    case 0x00: // fast PI mode block
    case 0x80: // block A
        pi_buffer[pi_pos] = buffer;
        for(uint8_t i=0; i<PI_BUFFER_SIZE; i++)
            if(pi_buffer[i]==pi_buffer[pi_pos])
                current_pi_count++;

        if(current_pi_count == 2 && !pi_checked)
        {
            Serial.print('P');
            serial_hex(pi_buffer[pi_pos] >> 8);
            serial_hex(pi_buffer[pi_pos] & 0xFF);
            Serial.print("?\n");
        }
        else if(current_pi_count > 2)
        {
            Serial.print('P');
            serial_hex(pi_buffer[pi_pos] >> 8);
            serial_hex(pi_buffer[pi_pos] & 0xFF);
            Serial.print('\n');
            pi_checked = true;
        }
        pi_pos = (pi_pos+1)%PI_BUFFER_SIZE;
        break;
    case 0x84: // block B
        // we will wait for block C & D before sending anything to the serial
        rds_buffer[0] = buffer >> 8;
        rds_buffer[1] = (uint8_t)buffer;
        rds_status_buffer = status&B11;
        rds_status_buffer |= B111100;
        rds_timer = millis();
        break;
    case 0x88: // block C
    case 0x90: // block C'
        rds_buffer[2] = buffer >> 8;
        rds_buffer[3] = (uint8_t)buffer;
        rds_status_buffer &= B0011;
        rds_status_buffer |= (status&B11) << 2;
        break;
    case 0x8C: // block D
        // is this block related to the block B from buffer?
        if((millis()-rds_timer) < 50)
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
        }
        break;
    }
}

void tune(boolean reset_rds_sync)
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
    
    if(reset_rds_sync && !scan_flag)
    {
        rds_sync_reset();
    }

    signal_reset();
}

bool tune_freq(uint32_t freq) // ***Modified by F4CMB***
{
    if ((freq>=55000) && (freq<=137000)) // FM BAND (extended)
    {
        if(freq % 50 || freq>108000 || scan_flag)
        {
            PLL = ((freq+10700)*2)/10;
            BAND = B00110001; // 5kHz step, fref=10kHz
        }
        else
        {
            // we use 50kHz step if possible, because
            // in other cases the audio output isn't clear
            PLL = ((freq+10700)*2)/100;
            BAND = B00100001; // 50kHz step, fref=100kHz
        }
    }
    else if ((freq>=100) && (freq<=1900)) // LW & MW BAND
    {
        BAND = B11101101;
        PLL = ((freq+10700)*20)/20;
    }
    else if ((freq>=1901) && (freq<=5899)) // AM SW 120m to 60m
    {
        BAND = B11010001;
        PLL = ((freq+10700)*16)/10;
    }
    else if ((freq>=5900) && (freq<=13870)) // AM SW 49m to 22m
    {
        BAND = B10110001;
        PLL = ((freq+10700)*10)/10;
    }
    else if ((freq>=13871) && (freq<=19020)) // AM SW 25m to 15m
    {
        BAND = B10010001;
        PLL = ((freq+10700)*8)/10;
    }
    else if ((freq>=19021) && (freq<=30000)) // AM SW 16m to 11m
    {
        BAND = B01110001;
        PLL = ((freq+10700)*6)/10;
    }
    else
    {
        return false;
    }

    align(freq);
    tune(true);
    return true;
}

void mode_FM()
{
    mode = MODE_FM;
    dsp_write_data(DSP_FM);
    dsp_set_filter(-1);
}

void mode_AM()
{
    mode = MODE_AM;
    dsp_write_data(DSP_AM);
}

void scan(bool continous)
{
    uint8_t _CONTROL = CONTROL;
    uint16_t _PLL = PLL;
    uint8_t _DAA = DAA;
    uint8_t _AGC = AGC;
    uint8_t _BAND = BAND; // save current settings
    uint32_t freq;

    scan_flag = true;
    dsp_write_16(DSP_VOLUME_SCALER, 0); // mute
    dsp_set_filter(scan_filter);
    tune_freq(scan_start);
    do
    {
        Serial.print('U');
        for(freq = scan_start; freq <= scan_end; freq += scan_step)
        {
            tune_freq(freq);
            Serial.print(get_current_freq(), DEC);
            Serial.print('=');
            serial_write_signal(signal_level(), 1);
            Serial.print(',');
        }
        Serial.print('\n');
    }
    while(continous && !Serial.available());

    scan_flag = false;

    // restore previous settings
    dsp_set_filter(current_filter);
    CONTROL = _CONTROL;
    PLL = _PLL;
    DAA = _DAA;
    AGC = _AGC;
    BAND = _BAND;
    tune(true);
    if(squelch_state)
    {
        dsp_write_16(DSP_VOLUME_SCALER, volume); // unmute
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

void ant_switch(uint8_t n)
{
    if(n < ANT_n)
    {
        digitalWrite(ANT[current_ant], LOW);
        current_ant = n;
        digitalWrite(ANT[current_ant], HIGH);
        signal_reset();
        delay(ANTENNA_SWITCH_DELAY);
        rds_sync_reset();
        Serial.print("z\n");
    }
}

float signal_level()
{
    float buffer;
    i2c.start(DSP_I2C | I2C_WRITE);
    if(mode == MODE_FM)
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
    {
        buffer = buffer * 0.797 + 3.5;
    }
    return buffer;
}

void serial_hex(uint8_t val)
{
    Serial.print(val >> 4 & 0xF, HEX);
    Serial.print(val & 0xF, HEX);
}

void serial_write_signal(float level, uint8_t precision)
{
    uint8_t n = (level-(int)level)*pow(10, precision);

    Serial.print((int)level, DEC);
    Serial.write('.');
    if(precision == 2 && n < 10)
    {
        Serial.write('0');
    }
    Serial.print(n, DEC);
}

void signal_reset()
{
    prev_level = 0.0;
    prev_stereo = false;
}

void rds_sync_reset()
{
    dsp_write_16(0x000035, 0x0060); // fast pi mode
    pi_checked = false;
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

/* IR support by F4CMB */
void sendcode(uint32_t code)
{
    cli();
    start();
    for (int i = 19; i >=0; i--)
    {
        if (code>>i &1 == 1) one();
        else zero();
    }
    delayMicroseconds (15000);
    start();
    for (int i = 19; i >=0; i--)
    {
        if (code>>i &1 == 1) one();
        else zero();
    }
    sei();
}

void carrier(int time)
{
    for (int i=0; i<(time/30); i++)
    {
        digitalWrite(IR_PIN, HIGH); // approx 40 KHz oscillator
        delayMicroseconds(12);
        digitalWrite(IR_PIN, LOW);
        delayMicroseconds(12);
    }
}

void start()
{
    carrier(2000);
    delayMicroseconds(600);
}

void one()
{
    carrier(1200);
    delayMicroseconds(600);
}

void zero()
{
    carrier(600);
    delayMicroseconds(600);
}
