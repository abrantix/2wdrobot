/*
  Level.h - Spirit level (bubble level) using the accelerometer.
*/

#ifndef __LEVEL_H
#define __LEVEL_H

#include "Game.h"

class Level : public Game
{
public:
  const char* Name() override { return "LEVEL"; }
  void Start() override { filtered[0] = filtered[1] = filtered[2] = 0; hasData = false; }
  void Frame(float dt) override;

private:
  float filtered[3];
  bool hasData = false;
};

extern Level LevelInstance;

#endif
