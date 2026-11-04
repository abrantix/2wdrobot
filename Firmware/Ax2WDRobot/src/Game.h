/*
  Game.h - Common interface of the little display games and the manager that cycles through them.
  While any game is active all motors are locked off.
*/

#ifndef __GAME_H
#define __GAME_H

#include <Arduino.h>

class Game
{
public:
  virtual ~Game() {}
  virtual const char* Name() = 0;
  virtual void Start() = 0;
  virtual void Frame(float dt) = 0;   // update + draw one frame, dt in seconds
};

class GameManager
{
public:
  void Next();                        // Pong -> Breakout -> Snake -> Level -> off
  bool IsActive() { return current >= 0; }
  void Process();

private:
  int current = -1;
  unsigned long titleUntil = 0;
  unsigned long lastFrame = 0;
  void Select(int index);
  void Stop();
  void DrawTitle(const char* name);
};

extern GameManager GameManagerInstance;

#endif
