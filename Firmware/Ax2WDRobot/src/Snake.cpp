#include "Snake.h"
#include "AxDisplay.h"
#include "Accelerometer.h"

Snake SnakeInstance;

static const int FIELD_TOP = 16;
static const int CELL = 4;
static const float TILT_THRESHOLD_G = 0.25f;
static const unsigned long STEP_MILLIS = 170;
static const unsigned long GAMEOVER_MILLIS = 3000;

void Snake::Start()
{
  length = 4;
  for(int i = 0; i < length; i++)
  {
    bodyX[i] = COLS / 2 - i;
    bodyY[i] = ROWS / 2;
  }
  dirX = 1;
  dirY = 0;
  score = 0;
  gameOver = false;
  lastStep = millis();
  PlaceFood();
}

bool Snake::OnBody(int x, int y)
{
  for(int i = 0; i < length; i++)
  {
    if(bodyX[i] == x && bodyY[i] == y)
    {
      return true;
    }
  }
  return false;
}

void Snake::PlaceFood()
{
  do
  {
    foodX = random(COLS);
    foodY = random(ROWS);
  } while(OnBody(foodX, foodY));
}

void Snake::ReadDirection()
{
  float x, y, z;
  if(!AccelerometerInstance.Read(x, y, z))
  {
    return;
  }

  // Raised side = direction: +x = up, +y = left (no U-turn)
  int nx = 0, ny = 0;
  if(fabsf(x) >= fabsf(y) && fabsf(x) > TILT_THRESHOLD_G)
  {
    ny = x > 0 ? -1 : 1;
  }
  else if(fabsf(y) > TILT_THRESHOLD_G)
  {
    nx = y > 0 ? -1 : 1;
  }
  if((nx || ny) && !(nx == -dirX && ny == -dirY))
  {
    dirX = nx;
    dirY = ny;
  }
}

void Snake::Step()
{
  int nx = (bodyX[0] + dirX + COLS) % COLS;
  int ny = (bodyY[0] + dirY + ROWS) % ROWS;

  bool eat = (nx == foodX && ny == foodY);
  // the tail moves away this step, so it is not an obstacle unless we grow
  int checkLen = eat ? length : length - 1;
  for(int i = 0; i < checkLen; i++)
  {
    if(bodyX[i] == nx && bodyY[i] == ny)
    {
      gameOver = true;
      stateSince = millis();
      return;
    }
  }

  if(eat && length < MAX_LEN)
  {
    length++;
    score++;
  }
  for(int i = length - 1; i > 0; i--)
  {
    bodyX[i] = bodyX[i - 1];
    bodyY[i] = bodyY[i - 1];
  }
  bodyX[0] = nx;
  bodyY[0] = ny;

  if(eat)
  {
    PlaceFood();
  }
}

void Snake::Frame(float dt)
{
  (void)dt;
  if(gameOver)
  {
    if(millis() - stateSince >= GAMEOVER_MILLIS)
    {
      Start();
    }
  }
  else
  {
    ReadDirection();
    if(millis() - lastStep >= STEP_MILLIS)
    {
      lastStep = millis();
      Step();
    }
  }
  Draw();
}

void Snake::Draw()
{
  Adafruit_SSD1306* d = AxDisplayInstance.BeginDraw();

  d->setCursor(0, 4);
  d->printf("Score %d", score);

  for(int i = 0; i < length; i++)
  {
    d->fillRect(bodyX[i] * CELL, FIELD_TOP + bodyY[i] * CELL, CELL - 1, CELL - 1, SSD1306_WHITE);
  }
  d->drawRect(foodX * CELL, FIELD_TOP + foodY * CELL, CELL - 1, CELL - 1, SSD1306_WHITE);

  if(gameOver)
  {
    d->fillRect(24, 28, 80, 14, SSD1306_BLACK);
    d->setCursor(34, 32);
    d->print("GAME OVER");
  }

  AxDisplayInstance.EndDraw();
}
