#include "Game.h"
#include "Pong.h"
#include "Breakout.h"
#include "Snake.h"
#include "Level.h"
#include "AxDisplay.h"
#include "MotorDriver.hpp"
#include "Trace.h"

GameManager GameManagerInstance;

static Game* games[] = { &PongInstance, &BreakoutInstance, &SnakeInstance, &LevelInstance };
static const int GAME_COUNT = sizeof(games) / sizeof(games[0]);
static const unsigned long FRAME_MILLIS = 20;
static const unsigned long TITLE_MILLIS = 900;

void GameManager::Next()
{
  if(current + 1 >= GAME_COUNT)
  {
    Stop();
  }
  else
  {
    Select(current + 1);
  }
}

void GameManager::Select(int index)
{
  if(current < 0)
  {
    TRACELN("Games: start, motors locked");
    MotorDriverInstance.SetLocked(true);
    AxDisplayInstance.SetGameMode(true);
  }
  current = index;
  DrawTitle(games[current]->Name());
  titleUntil = millis() + TITLE_MILLIS;
  games[current]->Start();
  lastFrame = 0;
}

void GameManager::Stop()
{
  TRACELN("Games: stop");
  current = -1;
  MotorDriverInstance.SetLocked(false);
  AxDisplayInstance.SetGameMode(false);
}

void GameManager::DrawTitle(const char* name)
{
  Adafruit_SSD1306* d = AxDisplayInstance.BeginDraw();
  d->setTextSize(2);
  int16_t x, y;
  uint16_t w, h;
  d->getTextBounds(name, 0, 0, &x, &y, &w, &h);
  d->setCursor((128 - w) / 2 - x, 26);
  d->print(name);
  AxDisplayInstance.EndDraw();
}

void GameManager::Process()
{
  if(current < 0)
  {
    return;
  }

  // Keep the motors off even if something unlocked them
  if(!MotorDriverInstance.IsLocked())
  {
    MotorDriverInstance.SetLocked(true);
  }

  unsigned long now = millis();
  if((long)(now - titleUntil) < 0 || now - lastFrame < FRAME_MILLIS)
  {
    return;
  }
  float dt = lastFrame ? (now - lastFrame) / 1000.0f : 0.02f;
  if(dt > 0.15f)
  {
    dt = 0.15f;
  }
  lastFrame = now;

  games[current]->Frame(dt);
}
