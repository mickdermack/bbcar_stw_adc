#pragma once

class Debouncer
{
public:
  Debouncer(int target_amount) : target_amount(target_amount)
  {
  }

  // Returns true if counter just reached target_amount
  bool update(bool state)
  {
    if (state && counter < target_amount)
    {
      counter++;

      if (counter == target_amount)
        return true;
    }
    else if (!state)
    {
      counter = 0;
    }

    return false;
  }

private:
  int target_amount;
  int counter;
};
