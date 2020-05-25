/***********************************************************************
File Name           : helpers.c
Project             : Self Balancing Robot
Author              : Sreejith S, Karthik
Date                : 08 Dec 2019
Platform            : Tiva Development Kit EK-TM4C123GXL
Descirption         : Misc helper functions
***********************************************************************/


#include "helpers.h"



intmax_t str_to_int(char* string)
{
  int a = 0; // Accumulator
  int  d; // Current digit
  int i = 0; // Array index
  char c = string[i]; // Current character

  i++; //Advance the index

  // Convert while not null terminator and is a numeric character
  while((c!=0)&&(c>='0')&&(c<='9'))
  {
    d = c - '0'; // Convert character to digit
    a = a * 10;  // Make room for digit
    a = a + d; // Add digit to accumulator
    c = string[i]; // Get next character
    i++; //Advance the index
  }
  return(a);
}


float str_to_float(char* string)
{
   int point_seen = 0;
   int d;
   
  float rez = 0, fact = 1;
  if (*string == '-'){
    string++;
    fact = -1;
  };
  for (point_seen = 0; *string; string++)
  {
    if (*string == '.'){
      point_seen = 1; 
      continue;
    };
    d = *string - '0';
    if (d >= 0 && d <= 9){
      if (point_seen) fact /= 10.0f;
      rez = rez * 10.0f + (float)d;
    };
  };
  return rez * fact;
}

