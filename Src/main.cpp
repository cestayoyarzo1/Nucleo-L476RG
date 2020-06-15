/*
  FILE: main.cpp
  Created on: June 14th, 2020, by Carlos Estay
  main source file
*/
#include "includes.h"

int main()
{
  System sys;
  sys.init();
  while(true)
  {
    sys.run();
  }
}
