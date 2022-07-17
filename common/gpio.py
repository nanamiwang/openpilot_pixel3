import os
PIXEL3 = os.path.isfile('/data/pixel3')

def gpio_init(pin: int, output: bool) -> None:
  if PIXEL3:
    return
  try:
    with open(f"/sys/class/gpio/gpio{pin}/direction", 'wb') as f:
      f.write(b"out" if output else b"in")
  except Exception as e:
    print(f"Failed to set gpio {pin} direction: {e}")


def gpio_set(pin: int, high: bool) -> None:
  if PIXEL3:
    return
  try:
    with open(f"/sys/class/gpio/gpio{pin}/value", 'wb') as f:
      f.write(b"1" if high else b"0")
  except Exception as e:
    print(f"Failed to set gpio {pin} value: {e}")
