#include "extendedNFC.h"

#ifndef NFC_NEW_LIB
#define NFC_NEW_LIB

int nfcReader::readText(String *s){
    uint16_t ret;
    NDEF_Text_info_t text = {NDEF_TEXT_UTF8, "", ""};
    sRecordInfo_t recordInfo;
    // increase buffer size for bigger messages
    ret = NDEF_ReadNDEF(NDEF_Buffer);
    if (ret) {
        return ret;
    }

    ret = NDEF_IdentifyBuffer(&recordInfo, NDEF_Buffer);
    if (ret) {
        return ret;
    }

    ret = NDEF_ReadText(&recordInfo, &text);
    if (ret) {
        return ret;
    }
    *s = String(text.text);

    return 0;
}

#endif