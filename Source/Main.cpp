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
  
  if (!app.initialize (argc, argv))
  {
	return -1;
  }
  
  app.run ();
  
  app.pending ();
  
  return 0;
}

