#include "bsp_buzzer.h"
#include "buzzer_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "detect_task.h"

extern TIM_HandleTypeDef htim4;
uint16_t sound_step = 5;
uint32_t counter = 0;
uint32_t sound_psc = 100;
uint32_t sound_arr = 250;
uint32_t sound_pwm = 90;
uint32_t test_variable = 0;
uint32_t delay_time = 0;
Buzzer_Note current_note;
uint32_t toe =0;

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t buzzer_high_water;
#endif
Music_Data xiaomi = {
    .gap = 30,
    .psc = 100,
    .pwm = 120,
    .music_length = 29,
    .music = XIAOMI,
};

Music_Data ddlc = {
    .gap = 150,
    .psc = 100,
    .pwm = 150,
    .music_length = 36,
    .music = DDLC,
};

Music_Data ied = {
    .gap = 50,
    .psc = 100,
    .pwm = 150,
    .music_length = 28,
    .music = IED,
};
Music_Data ckbk = {
    .gap = 50,
    .psc = 100,
    .pwm = 60,
    .music_length = 37,
    .music = CKBK,
};
Music_Data morse_test = {
    .gap = 150,
    .psc = 100,
    .pwm = 150,
    .music_length = 3,
    .music = MORSE_TEST,
};
Music_Data start = {
    .gap = 20,
    .psc = 100,
    .pwm = 100,
    .music_length = 3,
    .music = START,
};
Music_Data end = {
    .gap = 20,
    .psc = 100,
    .pwm = 100,
    .music_length = 3,
    .music = END,
};
Music_Data quinary_test = {
    .gap = 100,
    .psc = 100,
    .pwm = 60,
    .music_length = 4,
    .music = QUINARY_TEST,
};


void playMorseCode(int number) ;
void playMorseCodeDigit(char digit) ;
void buzzer_play(Music_Data music_data);
uint32_t toe_check(void);
void play_num_quinary(int number);
void play_toe_status(uint32_t toe_list);


void buzzer_play(Music_Data music_data)
{

    for (int i = 0; i < music_data.music_length; i++)
    {
        current_note.frequency = music_data.music[i][0];
        current_note.loudness = music_data.pwm;
        
        buzzer_set(music_data.psc,1000000/(music_data.music[i][0]),music_data.pwm);
        osDelay((uint32_t)(music_data.music[i][1]));
        buzzer_set(0,0,0);
        osDelay((uint32_t)(music_data.gap));
    }

}

void playMorseCodeDigit(char digit) {
    // Morse code patterns for digits 0-9
    const uint16_t morsePatterns[10][5] = {
        {MORSE_LONG, MORSE_LONG, MORSE_LONG, MORSE_LONG, MORSE_LONG}, // 0
        {MORSE_SHORT, MORSE_LONG, MORSE_LONG, MORSE_LONG, MORSE_LONG}, // 1
        {MORSE_SHORT, MORSE_SHORT, MORSE_LONG, MORSE_LONG, MORSE_LONG}, // 2
        {MORSE_SHORT, MORSE_SHORT, MORSE_SHORT, MORSE_LONG, MORSE_LONG}, // 3
        {MORSE_SHORT, MORSE_SHORT, MORSE_SHORT, MORSE_SHORT, MORSE_LONG}, // 4
        {MORSE_SHORT, MORSE_SHORT, MORSE_SHORT, MORSE_SHORT, MORSE_SHORT}, // 5
        {MORSE_LONG, MORSE_SHORT, MORSE_SHORT, MORSE_SHORT, MORSE_SHORT}, // 6
        {MORSE_LONG, MORSE_LONG, MORSE_SHORT, MORSE_SHORT, MORSE_SHORT}, // 7
        {MORSE_LONG, MORSE_LONG, MORSE_LONG, MORSE_SHORT, MORSE_SHORT}, // 8
        {MORSE_LONG, MORSE_LONG, MORSE_LONG, MORSE_LONG, MORSE_SHORT} // 9
    };

    if (digit < '0' || digit > '9') {
        return;
    }

    int index = digit - '0';
    const uint16_t* pattern = morsePatterns[index];

    for (int i = 0; i < 5; i++) {
        if (pattern[i] == 0) break; 
        buzzer_set(100,1000000/(pattern[i]),150);

        osDelay(500);
        buzzer_set(0,0,0);
        osDelay(100);
    }
    buzzer_set(100,1000000/(MORSE_GAP),150);
    osDelay(500);
    buzzer_set(0,0,0);
}

void playMorseCode(int number) {
    char str[12]; 
    sprintf(str, "%d", number);

    buzzer_play(start);
    for (size_t i = 0; i < strlen(str); i++) {
        playMorseCodeDigit(str[i]);
        buzzer_set(0,0,0);
        osDelay(500);
    }
    buzzer_play(end);
}
//13 = 55111 
void play_num_quinary(int number)
{
    if(number == 0)
    {
        buzzer_set(100,1000000/(QUINARY_0),60); 
        osDelay(300);
        
        buzzer_set(0, 0, 0);
    }
    else
    {
        while (number)
        {
            if ((number - 5) >= 0)
            {
                buzzer_set(100, 1000000 / (QUINARY_5), 60);
                osDelay(150);
                number -= 5;
            }
            else if ((number - 1) >= 0)
            {
                buzzer_set(100, 1000000 / (QUINARY_1), 60);
                osDelay(100);
                number -= 1;
            }
            buzzer_set(0, 0, 0);
            osDelay(150);
        }
    }

    osDelay(500);
}

uint32_t toe_check(void)
{
    uint32_t all_status = 0; 
    
    for (int i = 0; i < (ERROR_LIST_LENGTH); i++)
    {
        if (toe_is_error(i))
        {
            all_status |= (1 << i); 
        }
    }
    return all_status;
}

void play_toe_status(uint32_t toe_list)
{
    buzzer_play(quinary_test);

    for(int i =0; i < ERROR_LIST_LENGTH; i++)
    {
        if((toe_list >> i) & 1)
        {
            play_num_quinary(i);
        }
    }

    buzzer_play(end);
}
//
//buzzer_play(start);
void buzzer_task(void const *argument)
{
    toe = toe_check();
    play_toe_status(toe);
    while(1)
    {
        
        
#if INCLUDE_uxTaskGetStackHighWaterMark
	buzzer_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

