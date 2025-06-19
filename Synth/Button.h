#ifndef XVA1USERINTERFACE_BUTTON_H
#define XVA1USERINTERFACE_BUTTON_H


#include "Adafruit_MCP23017.h"

class Button {
public:
    // Function pointer definition
    typedef void (*onActionFunction)(Button *button, bool released);

    int id = 0;                             // ID for identification

    Button(Adafruit_MCP23017 *mcp, uint8_t buttonPin, int id, onActionFunction actionFunc);

    virtual void begin();

    void feedInput(uint16_t gpioAB);

    void process(int pinState);


    Adafruit_MCP23017 *getMcp() const;

protected:
    Adafruit_MCP23017 *mcp = nullptr;      // Pointer the I2C GPIO expander it's connected to
    uint8_t buttonPin = 0;
    onActionFunction actionFunc = nullptr; // Function pointer, will be called when there is an action happening

    int currentState;
    int lastButtonState;

    unsigned long lastDebounceTime = 0;     // The last time the output pin was toggled
    unsigned long debounceDelay = 10;       // The debounce time
};



#endif //XVA1USERINTERFACE_BUTTON_H
