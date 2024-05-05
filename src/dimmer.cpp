#include <Arduino.h>

unsigned short cutOfflinearizationLookup[] = {
	0,     398,  564,  691,  799,  894,  980, 1059, 1133, 1203, 1269, 1331, 1392, 1449, 1505, 1559, 
	1611, 1662, 1711, 1760, 1807, 1852, 1897, 1941, 1985, 2027, 2069, 2109, 2150, 2189, 2228, 2267, 
	2305, 2342, 2379, 2416, 2452, 2487, 2523, 2557, 2592, 2626, 2660, 2693, 2727, 2759, 2792, 2824, 
	2856, 2888, 2920, 2951, 2982, 3013, 3044, 3074, 3104, 3135, 3164, 3194, 3224, 3253, 3282, 3311, 
	3340, 3369, 3397, 3426, 3454, 3482, 3510, 3538, 3566, 3594, 3621, 3649, 3676, 3703, 3730, 3757, 
	3784, 3811, 3838, 3865, 3891, 3918, 3944, 3971, 3997, 4023, 4049, 4075, 4101, 4127, 4153, 4179, 
	4205, 4231, 4256, 4282, 4308, 4333, 4359, 4384, 4409, 4435, 4460, 4485, 4511, 4536, 4561, 4586, 
	4612, 4637, 4662, 4687, 4712, 4737, 4762, 4787, 4812, 4837, 4862, 4887, 4912, 4937, 4962, 4987, 
	5012, 5037, 5062, 5087, 5112, 5137, 5162, 5187, 5212, 5237, 5262, 5287, 5312, 5337, 5362, 5387, 
	5413, 5438, 5463, 5488, 5514, 5539, 5564, 5590, 5615, 5640, 5666, 5691, 5717, 5743, 5768, 5794, 
	5820, 5846, 5872, 5898, 5924, 5950, 5976, 6002, 6028, 6055, 6081, 6108, 6134, 6161, 6188, 6215, 
	6242, 6269, 6296, 6323, 6350, 6378, 6405, 6433, 6461, 6489, 6517, 6545, 6573, 6602, 6630, 6659, 
	6688, 6717, 6746, 6775, 6805, 6835, 6864, 6895, 6925, 6955, 6986, 7017, 7048, 7079, 7111, 7143, 
	7175, 7207, 7240, 7272, 7306, 7339, 7373, 7407, 7442, 7476, 7512, 7547, 7583, 7620, 7657, 7694, 
	7732, 7771, 7810, 7849, 7890, 7930, 7972, 8014, 8058, 8102, 8147, 8192, 8239, 8288, 8337, 8388, 
	8440, 8494, 8550, 8607, 8668, 8730, 8796, 8866, 8940, 9019, 9105, 9200, 9308, 9435, 9601, 10000
};

unsigned int zeroCrossCount = 0;

hw_timer_t *timer = NULL;


void IRAM_ATTR ignite() {
    digitalWrite(PIN_TRIAC, HIGH);
}

unsigned short cutOff = 10000;

unsigned long lastZeroTime = 0;


void IRAM_ATTR isr() {
	unsigned long time = micros();
	if (lastZeroTime > 0 && time - lastZeroTime < 9000)
    {
        return;
    }

    lastZeroTime = time;

    timerRestart(timer);

    if (cutOff != 0)
    {
        digitalWrite(PIN_TRIAC, LOW);
        if (cutOff < 10000)
        {
            timerAlarmWrite(timer, cutOff, false);
            timerAlarmEnable(timer);
        }
    }
    else
    {
        digitalWrite(PIN_TRIAC, HIGH);
    }
}

void dimmerBegin(uint8_t timerId) {
	pinMode(PIN_ZEROCROSS, INPUT_PULLDOWN);
	attachInterrupt(PIN_ZEROCROSS, isr, RISING);

	pinMode(PIN_TRIAC, OUTPUT);

	timer = timerBegin(timerId, 80, true);
	timerAttachInterrupt(timer, &ignite, true);
}

void dimmerSetLevel(uint8_t level)
{
    cutOff = cutOfflinearizationLookup[255 - level];
}

unsigned short dimmerPhase()
{
    return (unsigned short) timerRead(timer);
}