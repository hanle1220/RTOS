/*
 * string.h
 *
 *  Created on: Nov 12, 2021
 *      Author: giaha
 */

#ifndef STRING_H_
#define STRING_H_

#define MAX_CHARS 80
#define MAX_FIELDS 5

typedef struct _USER_DATA
{
char buffer[MAX_CHARS+1];
uint8_t fieldCount;
uint8_t fieldPosition[MAX_FIELDS];
char fieldType[MAX_FIELDS];
} USER_DATA;

void getsUart0(USER_DATA* data);
void parseFields(USER_DATA* data);
char* getFieldString(USER_DATA* data, uint8_t fieldNumber);
int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber);
bool strcompare(const char strCommand[], char string[]);
void strcopy(char* str1, char* str2);
void printFloat(uint32_t num);
void printHex(uint32_t num);
void printInt(uint32_t num);
void printBinary(uint32_t num);
void printString(char string[]);
void printState(uint8_t state);
uint32_t roundNum(float num);


#endif /* STRING_H_ */
