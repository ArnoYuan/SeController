/*
 * Main.cpp
 *
 *  Created on: 2016年8月17日
 *      Author: seeing
 */

#include "ControllerApplication.h"

using namespace NS_Controller;

int
main (int argc, char* argv[])
{
  ControllerApplication app;
  
  app.initialize ();
  
  app.run ();
  
  app.pending ();
  
  return 0;
}

