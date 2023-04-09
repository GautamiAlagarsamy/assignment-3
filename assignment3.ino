#ifndef __FREERTOS_H__
#define __FREERTOS_H__
 // Pin numbers
#define T1_PIN        10  // Task 1 output signal pin
#define T2_PIN        2   // Task 2 input measure signal pin
#define T3_PIN        3   // Task 3 input measure signal pin
#define T4_ANIN_PIN   5   // Task 4 analogue input pin
#define T4_LED_PIN    0   // Task 4 LED output pin
#define T6_PIN        4   // Task 6 push button input pin
#define T7_PIN        1   // Task 7 LED output pin
 // Task periods (ms)
#define TASK1_P       4     
#define TASK2_P       20    
#define TASK3_P       8     
#define TASK4_P       20    
#define TASK5_P       100   
#define TASK6_P       10    
#define TASK7_P       8     
// Task parameters
#define BAUD_RATE       9600  // Baud rate
#define TASK2_TIMEOUT   3100  // Timeout for pulseIn = worst case in us
#define TASK2_MINFREQ   333   // Task 2 minimum frequency of waveform in Hz
#define TASK2_MAXFREQ   1000  // Task 2 maximum frequency of waveform in Hz
 
#define TASK3_TIMEOUT   2100  // Timeout for pulseIn = worst case in us
#define TASK3_MINFREQ   500   // Task 3 minimum frequency of waveform in Hz
#define TASK3_MAXFREQ   1000  // Task 3 maximum frequency of waveform in Hz
#define NUM_PARAMS      4     // Task 4 length of array for storing past measurements
#define TASK4_THRESH    2048  // Task 4 threshold for turning LED on
#define TASK5_MIN       0     // Task 5 lower bound of range
#define TASK5_MAX       99    // Task 5 upper bound of range
// Helpful typedefs
typedef unsigned char uint8;
typedef unsigned int uint16;
typedef unsigned long uint32;
// Data structure for holding task 2 & 3 frequencies
typedef struct Freqs {
    double freq_t2;
    double freq_t3;
} Freqs;
 // Funtion Prototypes
void task1(void *pvParameters);
void task2(void *pvParameters);
void task3(void *pvParameters);
void task4(void *pvParameters);
void task5(void *pvParameters);
void task6(void *pvParameters);
void task7(void *pvParameters);
 #endif  // __FREERTOS_H__
 
// Convert period in us to frequency in Hz
#define periodToFreq_us(T) (1 / (T / 1000000))
 // Convert freeRTOS ticks to real time in ms
#define waitTask(t) (vTaskDelay(t / portTICK_PERIOD_MS))
 #if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif
 // Data structure & semaphore for storing task 2 & 3 frequencies
Freqs freqs;
SemaphoreHandle_t freqSem;
 QueueHandle_t btnQueue;
 uint16 anIn[NUM_PARAMS];  // Array for storing previous analogue measurements
uint8 currInd;            // Current index to overwrite in anIn[]
 void setup() {
  Serial.begin(BAUD_RATE);
   // Define pin inputs/outputs
  pinMode(T1_PIN, OUTPUT);
  pinMode(T2_PIN, INPUT);
  pinMode(T3_PIN, INPUT);
  pinMode(T4_ANIN_PIN, INPUT);
  pinMode(T4_LED_PIN, OUTPUT);
  pinMode(T6_PIN, INPUT);
  pinMode(T7_PIN, OUTPUT);
   // Initialise task 4 analogue input array with 0's
  currInd = 0;
  for (uint8 i = 0; i < NUM_PARAMS; i++) {
    anIn[i] = 0;
  }
   // Initialise task 2 & 3 frequencies to 0
  freqs.freq_t2 = 0;
  freqs.freq_t3 = 0;
   freqSem = xSemaphoreCreateMutex();
  btnQueue = xQueueCreate(1, sizeof(uint8));
   xTaskCreate(task1,"task1", 512,(void*) 1, 3, NULL);
   xTaskCreate(task2,"task2",512,(void*) 1, 2, NULL);
   xTaskCreate(task3,"task3",512,(void*) 1,3, NULL);
   xTaskCreate(task4,"task4",512,(void*) 1, 2, NULL);
   xTaskCreate(task5,"task5",512,(void*) 1,1, NULL);
   xTaskCreate(task6, "task6",512,(void*) 1,1, NULL);
   xTaskCreate(task7,"task7",512,(void*) 1,1, NULL);
}
 // Period = 4ms / Rate = 250Hz
void task1(void *pvParameters) {
  (void) pvParameters;
   for (;;) {
    // Generate waveform
    digitalWrite(T1_PIN, HIGH);
    delayMicroseconds(200);
    digitalWrite(T1_PIN, LOW);
    delayMicroseconds(50);
    digitalWrite(T1_PIN, HIGH);
    delayMicroseconds(30);
    digitalWrite(T1_PIN, LOW);
     waitTask(TASK1_P);
  }
}
// Period = 20ms / Rate = 50Hz
void task2(void *pvParameters) {
  (void) pvParameters;
 
  for (;;) {
    // Measure period when signal is HIGH
    // Multiply by 2 because 50% duty cycle
    double period = (double) pulseIn(T2_PIN, HIGH) * 2;
    double freqT2 = periodToFreq_us(period); // Convert to frequency using T = 1/f
     if(xSemaphoreTake(freqSem, portMAX_DELAY) == pdTRUE) {
      freqs.freq_t2 = freqT2;
      xSemaphoreGive(freqSem);
    }
   
    waitTask(TASK2_P);
  }
} 
// Period = 8ms / Rate = 125Hz
void task3(void *pvParameters) {
  (void) pvParameters;
 
  for (;;) {
    // Measure period when signal is HIGH
    // Multiply by 2 because 50% duty cycle
    double period = (double) pulseIn(T3_PIN, HIGH) * 2;
    double freqT3 = periodToFreq_us(period); // Convert to frequency using T = 1/f
 
    if(xSemaphoreTake(freqSem, portMAX_DELAY) == pdTRUE) {
      freqs.freq_t3 = freqT3;
      xSemaphoreGive(freqSem);
    }
 
    waitTask(TASK3_P);
  }
}
 
// Period = 20ms / Rate = 50Hz
void task4(void *pvParameters) {
  (void) pvParameters;
 
  for (;;) {
    // Read analogue signal and increment index for next reading
    // Analogue signal converted to 12 bit integer
    anIn[currInd] = analogRead(T4_ANIN_PIN);
    currInd = (currInd + 1) % NUM_PARAMS;
   
    // Sum array and divide to get average
    double filtAnIn = 0;
    for (uint8 i = 0; i < NUM_PARAMS; i++) {
      filtAnIn += anIn[i];
    }
    filtAnIn /= NUM_PARAMS;
 
    // Turn on LED if average is above half maximium of 12 bit integer (4096 / 2 = THRESH)
    digitalWrite(T4_LED_PIN, (filtAnIn > TASK4_THRESH));
 
    waitTask(TASK4_P);
  }
}
 
// Period = 100ms / Rate = 10Hz
void task5(void *pvParameters) {
  (void) pvParameters;
 
  for (;;) {
    if(xSemaphoreTake(freqSem, portMAX_DELAY) == pdTRUE) {
      // Map frequencies from 333Hz & 500Hz - 1000Hz to 0 - 99
      // Constrain to 0 - 99 as map() only creates gradient
      int normFreqT2 = map(freqs.freq_t2, TASK2_MINFREQ, TASK2_MAXFREQ, TASK5_MIN, TASK5_MAX);
      normFreqT2 = constrain(normFreqT2, TASK5_MIN, TASK5_MAX);
      int normFreqT3 = map(freqs.freq_t3, TASK3_MINFREQ, TASK3_MAXFREQ, TASK5_MIN, TASK5_MAX);
      normFreqT3 = constrain(normFreqT3, TASK5_MIN, TASK5_MAX);
 
      // Print to serial monitor
      Serial.print(normFreqT2);
      Serial.print(",");
      Serial.println(normFreqT3);
 
      xSemaphoreGive(freqSem);
    }
 
    waitTask(TASK5_P);
  }
}
 
// Period = 10ms / Rate = 100Hz
void task6(void *pvParameters) {
  (void) pvParameters;
 
  for (;;) {
    uint8 btn = digitalRead(T6_PIN);
    xQueueSend(btnQueue, &btn, 10);
 
    waitTask(TASK6_P);
  }
}
 
// Period = 8ms / Rate = 125Hz
void task7(void *pvParameters) {
  (void) pvParameters;
 
  for (;;) {
    uint8 btn = 0;
    if(xQueueReceive(btnQueue, &btn, 10) == pdPASS) {
      digitalWrite(T7_PIN, btn);
    }
     waitTask(TASK7_P);
  }
}
 void loop() {}
