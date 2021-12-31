#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include "uart0.h"
#include "string.h"
#include "gpio.h"
#include "tm4c123gh6pm.h"
#include <float.h>

uint32_t roundNum(float num)
{
    return (uint32_t)(num+0.5)*1;
}

void printBinary(uint32_t num)
{
      uint32_t i = 0x80000000;
      while(i)
      {
          putcUart0((num & i) ? '1' : '0');
          i >>= 1;
      }
}

void printHex(uint32_t num)
{
    char str[50];
    sprintf(str,"0x%08X\n",num);
    putsUart0(str);
}

void printInt(uint32_t num)
{
    char str[50];
    sprintf(str,"%d",num);
    putsUart0(str);
}

void printFloat(uint32_t num)
{
    printInt(num/100);
    putcUart0('.');
    putcUart0((char) (num % 10 +'0'));
    putcUart0((char) (num/10 % 10 +'0'));
}

void printString(char string[])
{
    char str[50];
    sprintf(str,"%-10s",string);
    putsUart0(str);
}

void printState(uint8_t state)
{
    switch(state)
    {
        case 0:
            putsUart0("INVALID");
            break;
        case 1:
            putsUart0("UNRUN");
            break;
        case 2:
            putsUart0("READY");
            break;
        case 3:
            putsUart0("DELAYED");
            break;
        case 4:
            putsUart0("BLOCKED");
            break;
        case 5:
            putsUart0("SUSPENDED");
            break;
    }
}

void getsUart0(USER_DATA* data)
{
    char c;
    uint8_t count = 0;
    while(true)
    {
        if(count == MAX_CHARS)
        {
            data->buffer[count] ='\0';
            break;
        }
        c = getcUart0();
        if(c == 8 || c == 127)
        {
            count--;
        }
        else if(c == 13 || c == 10)
        {
            data->buffer[count] ='\0';
            break;
        }
        else if(c >= 32)
        {
            data->buffer[count] = c;
            count++;
        }
    }
    return;
}

void parseFields(USER_DATA* data)
{
    uint8_t pos = 0;
    data->fieldCount = 0;
    while(data->buffer[pos] != '\0')
    {
        if(data->fieldCount > MAX_FIELDS)
        {
            break;
        }
        if(isalpha(data->buffer[pos]) != 0)
        {
             data->fieldType[data->fieldCount] = 'a';
             data->fieldPosition[data->fieldCount] = pos;
             if(data->buffer[pos-1] == '\0')
                 data->fieldCount++;
        }
        else if(isdigit(data->buffer[pos]) != 0)
        {
             data->fieldType[data->fieldCount] = 'n';
             data->fieldPosition[data->fieldCount] = pos;
             if(data->buffer[pos-1] == '\0')
                 data->fieldCount++;
        }
        else if(data->buffer[pos] == 38)
        {
             data->fieldType[data->fieldCount] = 's';
             data->fieldPosition[data->fieldCount] = pos;
             if(data->buffer[pos-1] == '\0')
                 data->fieldCount++;
        }
        else
        {
            data->buffer[pos] = '\0';
        }
        pos++;
    }
}

char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
    if(fieldNumber <= data->fieldCount)
    {
        if(data->fieldType[fieldNumber] == 'a')
        {
            return &data->buffer[data->fieldPosition[fieldNumber]];
        }
        else
            return 0;
    }
    else
        return 0;
}

int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    char str[50] = " ";
    if(data->fieldType[fieldNumber] == 'n')
    {
        strcpy(str, &data->buffer[data->fieldPosition[fieldNumber]]);

        int i = 0, num = 0;
        while(str[i] != '\0')
        {
            num = num*10+(str[i]-48);
            i++;
        }
        return num;
    }
    else
        return 0;
}

bool strcompare(const char strCommand[], char string[])
{
    uint8_t i = 0;
    if(string[i] == '\0') return false;
    while(string[i] != '\0')
    {
        if(strCommand[i] != string[i])
            return false;
        i++;
    }
    return true;
}

void strcopy(char* str1, char* str2)
{
    uint8_t k;
    for (k = 0; str1[k] != '\0'; k++)
       str2[k] = str1[k];
    str2[k] = '\0';
}
