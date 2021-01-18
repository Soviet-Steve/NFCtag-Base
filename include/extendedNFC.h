#include "ST25DVSensor.h"

class nfcReader : public ST25DV{
    public:
        int readText(String *s);
        int writeText(String message);
};

extern nfcReader nfc;