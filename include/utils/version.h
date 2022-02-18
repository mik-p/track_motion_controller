#pragma once

#include <Arduino.h>

#ifndef VERSION
#define VERSION "0.0.0"
#endif

#ifndef RELEASE_NAME_STR
#define RELEASE_NAME_STR "unknown"
#endif

/**
 * @brief print firmware version details to the given stream interface
 *
 */
void print_version(Stream* interface_)
{
  interface_->println();
  interface_->print("firmware: ");
  interface_->print(__FILE__);
  interface_->print(" v");
  interface_->println(VERSION);
  interface_->print(__DATE__);
  interface_->print(" ");
  interface_->println(__TIME__);
  interface_->print("name: ");
  interface_->println(RELEASE_NAME_STR);
}
