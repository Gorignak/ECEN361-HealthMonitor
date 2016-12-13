/*
 modified to fulfill lab7.
*/
/* FreeRTOS.org includes. */
#include "FreeRTOS_AVR.h"
//#include "task.h"
//#include "semphr.h"
//#include "portasm.h"

/* LCD Display include. */
#include <LiquidCrystal.h>

/* Demo includes. */
#include "basic_io_avr.h"

/* Compiler includes. */
//#include <dos.h>

#include <avr/interrupt.h>
#include <math.h>

/* LCD Library Initialization */
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);


/* The tasks to be created. */
static void vPulseRead( void *pvParameters );
static void vTempRead( void *pvParameters );
static void vDisplayReadings( void *pvParameters);

void interruptSetup();



/*-----------------------------------------------------------*/

/* Declare a variable of type SemaphoreHandle_t.  This is used to reference the
semaphore that is used to synchronize a task with an interrupt. */
SemaphoreHandle_t xCountingSemaphore;

// Queue for keeping track of data
QueueHandle_t xQueue;

int pulsePin = 0;                 // Pulse Sensor red wire connected to analog pin 0
volatile int BPM;                   // int that holds raw Analog in 0. updated every 2mS


//temperature
double TempDisp;


void setup( void )
{
  Serial.begin(9600);
  // set up the LCD's number of columns and rows:
  pinMode(5, OUTPUT);
  analogWrite(5, 50);
  lcd.begin(16, 2);
  // Setup LCD Screen
  lcd.setCursor(0, 0);
  lcd.print("HR:");
  lcd.setCursor(8, 0);
  lcd.print("TEMP:");
  /* Before a semaphore is used it must be explicitly created.  In this example
  a counting semaphore is created.  The semaphore is created to have a maximum
  count value of 10, and an initial count value of 0. */
  xCountingSemaphore = xSemaphoreCreateCounting( 10, 0 );

  //Create the queue for passing button pushes
  xQueue = xQueueCreate(1, sizeof( long) );

  /* Install the interrupt handler. */
//  _dos_setvect( 0x82, vExampleInterruptHandler );



  /* Check the semaphore was created successfully. */
  if( xCountingSemaphore != NULL )
  {
    /* Create the 'handler' task.  This is the task that will be synchronized
    with the 2ms timer interrupt.  The handler task is created with a high priority to
    ensure it runs immediately after the interrupt exits.  In this case a
    priority of 3 is chosen. */
    xTaskCreate( vPulseRead, "PulseRead", 200, NULL, 3, NULL );

    /* Create the task that will periodically generate a software interrupt.
    This is created with a priority below the handler task to ensure it will
    get preempted each time the handler task exist the Blocked state. */
    xTaskCreate( vTempRead, "TempRead", 200, NULL, 2, NULL );

    /* Create the display task for the LCD screen. */
    xTaskCreate( vDisplayReadings, "DisplayReadings", 200, NULL, 2, NULL );
    
    // Set up interrupts
    interruptSetup();
    
    /* Start the scheduler so the created tasks start executing. */
    vTaskStartScheduler();
  }

    /* If all is well we will never reach here as the scheduler will now be
    running the tasks.  If we do reach here then it is likely that there was
    insufficient heap memory available for a resource to be created. */
  for( ;; );
//  return 0;
}
/*-----------------------------------------------------------*/
static void vDisplayReadings( void *pvParameters )
{
  int counter = 0;
  float useTemp;
  for( ;; )
  {
    useTemp = TempDisp;
    lcd.setCursor(0, 1);
    lcd.print(BPM);
    lcd.setCursor(8, 1);
    lcd.print(useTemp);

   // counter++;
   // if(counter == 200)
  //  {
      
    Serial.print("Heart Rate:");
    Serial.println(BPM, DEC);
    Serial.print("    Temp:");
    Serial.print(useTemp, DEC);
    //Serial.println(step);
   // counter = 0;
   // }

    delay(100);
  }
}

static void vPulseRead( void *pvParameters )
{
  // Set up queue variable
  portBASE_TYPE xStatus;

  // Pulse sensor variables
  
  volatile int rate[10];                    // array to hold last ten IBI values
  volatile unsigned long sampleCounter = 0;          // used to determine pulse timing
  volatile unsigned long lastBeatTime = 0;           // used to find IBI
  volatile int P = 512;                      // used to find peak in pulse wave, seeded
  volatile int T = 512;                     // used to find trough in pulse wave, seeded
  volatile int thresh = 525;                // used to find instant moment of heart beat, seeded
  volatile int amp = 100;                   // used to hold amplitude of pulse waveform, seeded
  volatile boolean firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
  volatile boolean secondBeat = false;      // used to seed rate array so we startup with reasonable BPM

  
  volatile int Signal;                // holds the incoming raw data
  volatile int IBI = 600;             // int that holds the time interval between beats! Must be seeded! 
  volatile boolean Pulse = false;     // "True" when User's live heartbeat is detected. "False" when not a "live beat". 
  volatile boolean QS = false;        // becomes true when Arduoino finds a beat.
  
  /* As per most tasks, this task is implemented within an infinite loop. */
  for( ;; )
  {
    /* Use the semaphore to wait for the event.  The semaphore was created
    before the scheduler was started so before this task ran for the first
    time.  The task blocks indefinitely meaning this function call will only
    return once the semaphore has been successfully obtained - so there is no
    need to check the returned value. */
    xSemaphoreTake( xCountingSemaphore, portMAX_DELAY );

    /* To get here the event must have occurred.  Process the event. */
    cli();                                      // disable interrupts while we do this
    Signal = analogRead(pulsePin);              // read the Pulse Sensor 
    sampleCounter += 2;                         // keep track of the time in mS with this variable
    int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise

    //  find the peak and trough of the pulse wave
    if(Signal < thresh && N > (IBI/5)*3)       // avoid dichrotic noise by waiting 3/5 of last IBI
    {
      if (Signal < T)                        // T is the trough
      {
        T = Signal;                         // keep track of lowest point in pulse wave 
      }
    }

    if(Signal > thresh && Signal > P)          // thresh condition helps avoid noise
    {
      P = Signal;                             // P is the peak
    }                                        // keep track of highest point in pulse wave

  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
  // signal surges up in value every time there is a pulse
    if (N > 250)                                   // avoid high frequency noise
    {
      if ( (Signal > thresh) && (Pulse == false) && (N > (IBI/5)*3) )
      {        
        Pulse = true;                               // set the Pulse flag when we think there is a pulse
        IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
        lastBeatTime = sampleCounter;               // keep track of time for next pulse

        if(secondBeat)                        // if this is the second beat, if secondBeat == TRUE
        {
          secondBeat = false;                  // clear secondBeat flag
          for(int i=0; i<=9; i++)             // seed the running total to get a realisitic BPM at startup
          {
            rate[i] = IBI;                      
          }
        }

        if(firstBeat)                         // if it's the first time we found a beat, if firstBeat == TRUE
        {
          firstBeat = false;                   // clear firstBeat flag
          secondBeat = true;                   // set the second beat flag
          sei();                               // enable interrupts again
        }                                      // IBI value is unreliable so discard it  
        
        else                                  // IBI value is good because this is NOT the first beat.
        {
        // keep a running total of the last 10 IBI values
        word runningTotal = 0;                  // clear the runningTotal variable    

        for(int i=0; i<=8; i++)                // shift data in the rate array
        {
          rate[i] = rate[i+1];                  // and drop the oldest IBI value 
          runningTotal += rate[i];              // add up the 9 oldest IBI values
        }

        rate[9] = IBI;                          // add the latest IBI to the rate array
        runningTotal += rate[9];                // add the latest IBI to runningTotal
        runningTotal /= 10;                     // average the last 10 IBI values 
        BPM = 60000/runningTotal;               // how many beats can fit into a minute? that's BPM!
        QS = true;                              // set Quantified Self flag 
        }                       
      }
    }

    if (Signal < thresh && Pulse == true)   // when the values are going down, the beat is over
    {
      Pulse = false;                         // reset the Pulse flag so we can do it again
      amp = P - T;                           // get amplitude of the pulse wave
      thresh = amp/2 + T;                    // set thresh at 50% of the amplitude
      P = thresh;                            // reset these for next time
      T = thresh;
    }

    if (N > 2500)                           // if 2.5 seconds go by without a beat
    {
      thresh = 512;                          // set thresh default
      P = 512;                               // set P default
      T = 512;                               // set T default
      lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date        
      firstBeat = true;                      // set these to avoid noise
      secondBeat = false;                    // when we get the heartbeat back
    }
  
    sei();                                   // enable interrupts when youre done reading!

    if (QS == true)     // A Heartbeat Was Found
    {                   // BPM and IBI have been Determined
                        // Quantified Self "QS" true when arduino finds a heartbeat

      /*                 
      Serial.print("*** Heart-Beat Happened *** ");
      Serial.print("BPM: ");
      Serial.print(BPM);
      Serial.println("  ");    
      */
      QS = false;                      // reset the Quantified Self flag for next time    
    }
  }// end for loop
}// end task

/*-----------------------------------------------------------*/

static void vTempRead( void *pvParameters )
{
  // Set up queue variable
  portBASE_TYPE xStatus;

  //Temp Reading variables
  long Resistance; double Temp;    // Dual-Purpose variable to save space.
  int RawADC;
  
  #define ThermistorPIN 1         // Analog pin 1
  /* As per most tasks, this task is implemented within an infinite loop. */
  for( ;; )
  {
    RawADC = analogRead(ThermistorPIN);

    // Inputs ADC Value from Thermistor and outputs Temperature in Celsius
    //  requires: include <math.h>
    // Utilizes the Steinhart-Hart Thermistor Equation:
    //    Temperature in Kelvin = 1 / {A + B[ln(R)] + C[ln(R)]^3}
    //    where A = 0.001129148, B = 0.000234125 and C = 8.76741E-08

    
    Resistance=10000.0*((1024.0/(1025-RawADC) - 1));  // Assuming a 10k Thermistor.  Calculation is actually: Resistance = (1024 /ADC -1) * BalanceResistor
    // For a GND-Thermistor-PullUp--Varef circuit it would be Rtherm=Rpullup/(1024.0/ADC-1)
    Temp = log(Resistance); // Saving the Log(resistance) so not to calculate it 4 times later. // "Temp" means "Temporary" on this line.
    Temp = 1 / (0.001129148 + (0.000234125 * Temp) + (0.0000000876741 * Temp * Temp * Temp));   // Now it means both "Temporary" and "Temperature"
    Temp = Temp - 273.15;  // Convert Kelvin to Celsius                                         // Now it only means "Temperature"

    TempDisp = Temp;
    // Uncomment this line to convert to Fahrenheit instead.
    //Temp = (Temp * 9.0)/ 5.0 + 32.0; // Convert to Fahrenheit

    //delay(100);                                           // Delay a bit to not Serial.print faster than the serial connection can output

  }
}


/*---------------------------------------------------------*/
void interruptSetup(){     
  // Initializes Timer2 to throw an interrupt every 2mS.
  TCCR2A = 0x02;     // DISABLE PWM ON DIGITAL PINS 3 AND 11, AND GO INTO CTC MODE
  TCCR2B = 0x06;     // DON'T FORCE COMPARE, 256 PRESCALER 
  OCR2A = 0X7C;      // SET THE TOP OF THE COUNT TO 124 FOR 500Hz SAMPLE RATE
  TIMSK2 = 0x02;     // ENABLE INTERRUPT ON MATCH BETWEEN TIMER2 AND OCR2A
  sei();             // MAKE SURE GLOBAL INTERRUPTS ARE ENABLED      
} 

/*-----------------------------------------------------------*/

// Interrupt Service Routine attached to 2ms timer, allows pulseRead task to run
ISR(TIMER2_COMPA_vect)
{
static portBASE_TYPE xHigherPriorityTaskWoken;

  //Pulse Read Semaphore Stuff
  xHigherPriorityTaskWoken = pdFALSE;

  /* 'Give' the semaphore to unblock the handler task. */
  xSemaphoreGiveFromISR( xCountingSemaphore, (BaseType_t*)&xHigherPriorityTaskWoken );

  if( xHigherPriorityTaskWoken == pdTRUE )
  {
    /* Giving the semaphore unblocked a task, and the priority of the
    unblocked task is higher than the currently running task - force
    a context switch to ensure that the interrupt returns directly to
    the unblocked (higher priority) task.

    NOTE: The syntax for forcing a context switch is different depending
    on the port being used.  Refer to the examples for the port you are
    using for the correct method to use! */
    // portSWITCH_CONTEXT();
    vPortYield(); 
  }
}


//---------------------------------------------------------------
void loop() {}
