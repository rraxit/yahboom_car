#include <Arduino.h>
#define BAUD_RATE 115200
#define LED_BUILTIN 2

TaskHandle_t Task1;
TaskHandle_t Task2;
int count = 0;

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
bool serialEvent(String& inputString) 
{

  while (Serial2.available()) {
    // get the new byte:
    char inChar = (char)Serial2.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      return true;
    }
  }
  return false; 
}


//Task1code: blinks an LED every 1000 ms
void Task1code( void * pvParameters ){
  Serial.print("Serial2 sender running on core ");
  Serial.println(xPortGetCoreID());
  // send serial message 
  uint32_t wait = 1000; //ms
  
  do{

    Serial2.printf("ESP32 sends message %d times \n", ++count);
    digitalWrite(LED_BUILTIN, LOW);    
    vTaskDelay(pdMS_TO_TICKS(wait));
  }while(true);


}

//Task2code: blinks an LED every 700 ms
void Task2code( void * pvParameters ){
  Serial.print("Serial2 receiever running on core ");
  Serial.println(xPortGetCoreID());

  // receive serial message 
  uint32_t wait = 100; //ms
  String inputString = "";  
  do{

    if(serialEvent(inputString))
    {
      Serial.println(inputString);
      inputString = "";  
      digitalWrite(LED_BUILTIN, HIGH);
    }
    vTaskDelay(pdMS_TO_TICKS(wait));
  }while(true);
}


void setup()
{
  // inititalize serial communication 
  Serial.begin(BAUD_RATE);
  Serial2.begin(BAUD_RATE);

  // LED config 
  pinMode(LED_BUILTIN, OUTPUT);

  // //wait unitil serial communication established 
  // while(!Serial)
  //   delay(100);

  // multithread 

   //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
  delay(500); 
}

void loop()
{

}