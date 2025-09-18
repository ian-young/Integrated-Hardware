#include <Arduino.h>
#include <pgmspace.h> // Required for PROGMEM

// Pin definitions
#define HIT_SENSOR_PIN D1
#define RED_PIN        D5
#define GREEN_PIN      D6
#define BLUE_PIN       D7

// Configuration
#define TIMEOUT_MS     3000 // Max time between knocks
#define TOLERANCE_MS   100  // Timing tolerance for knocks
#define DISPLAY_TIME_MS 3000 // How long to show success/failure light

// --- Advanced Concept: Ring Buffer (Circular Queue) ---
// This queue is used to safely pass knock timestamps from the
// interrupt service routine (ISR) to the main loop.
#define QUEUE_SIZE 16 // Must be a power of 2 for efficient modulo
volatile unsigned long knockQueue[QUEUE_SIZE];
volatile uint8_t queueHead = 0; // Index for writing (only modified by ISR)
volatile uint8_t queueTail = 0; // Index for reading (only modified by main loop)

// Storing the secret pattern in Program Memory (Flash)
// This saves RAM, which is important for the ESP8266.
const int secretPattern[] PROGMEM = { 250, 600, 250 }; // Sequence of 4 knocks
const int patternLength = sizeof(secretPattern) / sizeof(secretPattern[0]);

// Global variables for the state machine to avoid `static` issues
unsigned long lastKnockTime = 0;
int patternIndex = 0;

// Function Pointer for the State Machine
// This allows the main loop to call the correct state handler function.
void (*currentStateFunction)();

// Forward declarations of state and helper functions
void handleIdleState();
void handleListeningState();
void handleSuccessState();
void handleFailureState();
bool queueRead(unsigned long* knockTime);
void setLedColor(int r, int g, int b);

// =================================================================
// INTERRUPT SERVICE ROUTINE (ISR)
// =================================================================
// The ISR is called every time a knock is detected (FALLING edge).
// It's crucial to do as little as possible here. We just record the time
// and add it to the ring buffer.
void IRAM_ATTR handleKnock() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  
  if (interruptTime - lastInterruptTime > 100) { // <--- Adjusted debounce to 100ms
    Serial.println("ISR: Knock detected."); // <-- VERBOSE
    // --- Writing to the Ring Buffer ---
    // Calculate the next head index using a bitwise AND for speed
    uint8_t nextHead = (queueHead + 1) & (QUEUE_SIZE - 1);

    // Check if the queue is full. If so, we discard the new knock
    // to prevent overwriting old data.
    if (nextHead != queueTail) {
      knockQueue[queueHead] = interruptTime;
      queueHead = nextHead;
    }
  }
  lastInterruptTime = interruptTime;
}

// =================================================================
// SETUP and LOOP
// =================================================================
void setup() {
  // Set a standard baud rate for the Serial Monitor
  Serial.begin(115200); 
  pinMode(HIT_SENSOR_PIN, INPUT_PULLUP); // <--- Use INPUT_PULLUP
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  // Attach the ISR to the hit sensor pin
  attachInterrupt(digitalPinToInterrupt(HIT_SENSOR_PIN), handleKnock, FALLING);

  // Initialize the state machine to the idle state
  currentStateFunction = &handleIdleState;
  Serial.println("Queue-Based Knock Detector Initialized.");
  Serial.println("Device is ready!");
}

void loop() {
  // The main loop just calls the current state function.
  // This is the heart of the state machine pattern.
  currentStateFunction();
}

// =================================================================
// STATE HANDLER FUNCTIONS
// =================================================================
/**
 * State 1: IDLE
 * The system is waiting for the first knock to start the pattern.
 * The LED is off.
 */
void handleIdleState() {
  setLedColor(0, 0, 0); // LED Off
  unsigned long firstKnockTime;
  
  // Check if there's a knock in the queue.
  if (queueRead(&firstKnockTime)) {
    Serial.println("EVENT: First knock detected."); 
    
    // Reset global state variables and set up for listening
    lastKnockTime = firstKnockTime;
    patternIndex = 0;
    
    Serial.print("INFO: First knock time is "); Serial.println(firstKnockTime);
    
    setLedColor(255, 200, 0); // Yellow "listening" color
    currentStateFunction = &handleListeningState;
  }
}

/**
 * State 2: LISTENING
 * The system is actively waiting for subsequent knocks and checking their timing
 * against the secret pattern.
 */
void handleListeningState() {
  unsigned long currentKnockTime;
  
  // Check for a timeout since the LAST valid knock.
  if (millis() - lastKnockTime > TIMEOUT_MS) {
    Serial.println("EVENT: Timeout reached. Transitioning to Failure."); 
    currentStateFunction = &handleFailureState;
    return;
  }

  // Check if there's a new knock in the queue to process.
  if (queueRead(&currentKnockTime)) {
    Serial.print("EVENT: Subsequent knock detected at "); Serial.println(currentKnockTime);
    unsigned long interval = currentKnockTime - lastKnockTime;
    // Read the expected interval from PROGMEM
    unsigned int expectedInterval = pgm_read_word(&secretPattern[patternIndex]);

    // Added Serial print to show the interval for debugging
    Serial.print("Knock interval: ");
    Serial.println(interval);

    // Use abs() by casting the unsigned long difference to a signed long.
    long diff = (long)interval - (long)expectedInterval;
    
    if (abs(diff) <= TOLERANCE_MS) {
      Serial.print("RESULT: Correct interval. "); 
      Serial.print("Expected: "); 
      Serial.println(expectedInterval);
      lastKnockTime = currentKnockTime;
      patternIndex++;

      // Check if the entire pattern has been successfully matched.
      if (patternIndex >= patternLength) {
        Serial.println("RESULT: Entire pattern matched.");
        currentStateFunction = &handleSuccessState;
      }
    } else {
      // Incorrect knock, so we transition to failure.
      Serial.print("RESULT: Wrong interval. "); 
      Serial.print("Expected: "); 
      Serial.println(expectedInterval);
      currentStateFunction = &handleFailureState;
    }
  }
}

/**
 * State 3: SUCCESS
 * Displays a green light for a set duration, then returns to idle.
 */
void handleSuccessState() {
  static unsigned long displayStartTime = 0;

  if (displayStartTime == 0) {
    Serial.println("ACCESS GRANTED");
    setLedColor(0, 255, 0); // Green
    // Flush the queue to ignore any knocks that happened while the
    // success state was active.
    while(queueRead(nullptr));
    displayStartTime = millis();
  }

  if (millis() - displayStartTime > DISPLAY_TIME_MS) {
    displayStartTime = 0;
    currentStateFunction = &handleIdleState;
  }
}

/**
 * State 4: FAILURE
 * Displays a red light for a set duration, then returns to idle.
 */
void handleFailureState() {
  static unsigned long displayStartTime = 0;

  if (displayStartTime == 0) {
    Serial.println("ACCESS DENIED");
    setLedColor(255, 0, 0); // Red
    // Flush the queue to ignore lingering knocks from the failed attempt.
    while(queueRead(nullptr));
    displayStartTime = millis();
  }

  if (millis() - displayStartTime > DISPLAY_TIME_MS) {
    displayStartTime = 0;
    currentStateFunction = &handleIdleState;
  }
}

// =================================================================
// HELPER FUNCTIONS
// =================================================================
/**
 * Reads one item from the queue.
 * @param knockTime A pointer to store the read timestamp. Can be nullptr to just discard.
 * @return True if an item was read, false if the queue was empty.
 */
bool queueRead(unsigned long* knockTime) {
  if (queueHead == queueTail) {
    return false; // Queue is empty
  }

  // Read the data from the tail
  if (knockTime != nullptr) {
    *knockTime = knockQueue[queueTail];
  }

  // Move the tail pointer
  queueTail = (queueTail + 1) & (QUEUE_SIZE - 1);
  return true;
}

/**
 * Helper function to set the RGB LED color using PWM.
 * @param r Red value (0-255)
 * @param g Green value (0-255)
 * @param b Blue value (0-255)
 */
void setLedColor(int r, int g, int b) {
  // The ESP8266 analogWrite uses a 10-bit resolution (0-1023)
  analogWrite(RED_PIN, map(r, 0, 255, 0, 1023));
  analogWrite(GREEN_PIN, map(g, 0, 255, 0, 1023));
  analogWrite(BLUE_PIN, map(b, 0, 255, 0, 1023));
}
