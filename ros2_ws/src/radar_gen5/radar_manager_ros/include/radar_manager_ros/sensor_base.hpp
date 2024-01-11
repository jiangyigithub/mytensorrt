/**
 * Copyright (c) 2009, 2020 Robert Bosch GmbH and its subsidiaries.
 * This program and the accompanying materials are made available under
 * the terms of the Bosch Internal Open Source License v4
 * which accompanies this distribution, and is available at
 * http://bios.intranet.bosch.com/bioslv4.txt
 */

#ifndef SENSOR_BASE_HPP_
#define SENSOR_BASE_HPP_

class CSensorBase
{
  /* This is the SensorBase class.
   * All individual sensor should implement/derive this class
   */

public:
  CSensorBase() = default;
  virtual ~CSensorBase()
  {
  }

  virtual void Init() = 0;
  virtual void Start() = 0;
  virtual void Stop() = 0;
};

#endif
