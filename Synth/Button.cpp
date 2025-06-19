#include "Button.h"

/*
 * Constructor
 */
Button::Button(Adafruit_MCP23017 *mcp, uint8_t buttonPin, int id, onActionFunction actionFunc) {
    this->mcp = mcp;
    this->buttonPin = buttonPin;
    this->id = id;
    this->actionFunc = actionFunc;
    currentState = HIGH;
    lastButtonState = HIGH;
}

void Button::begin() {
    mcp->pinMode(buttonPin, INPUT);
    mcp->pullUp(buttonPin, HIGH);     // Pulled high ~100k

    currentState = mcp->digitalRead(buttonPin);
}

void Button::feedInput(uint16_t gpioAB) {
    uint8_t pinState = bitRead(gpioAB, buttonPin);
    process(pinState);
}

void Button::process(int pinState) {
    if (pinState != lastButtonState) {
        // If the switch changed, due to noise or pressing:
        lastDebounceTime = millis();
    }

    unsigned long time = millis() - lastDebounceTime;

    if (time > debounceDelay) {
        // whatever the reading is at, it's been there for longer than the debounce
        // delay, so take it as the actual current state

        if (pinState != currentState) {
            // The button state has been changed:
            currentState = pinState;
            bool released = pinState == HIGH;

            // Call action function if registered
            if (actionFunc) {
                actionFunc(this, released);
            }
        }

    }

    lastButtonState = pinState;
}

Adafruit_MCP23017 *Button::getMcp() const {
    return mcp;
}
