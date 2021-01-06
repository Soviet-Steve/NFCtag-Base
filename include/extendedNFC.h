#include "ST25DVSensor.h"

class nfcReader : public ST25DV{
    public:
        int readText(String *s);
};
