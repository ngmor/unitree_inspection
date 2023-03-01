# Raspberry Pi Pico

import board,busio
from time import sleep
from adafruit_bitmap_font import bitmap_font
from adafruit_st7735r import ST7735R
import displayio
from digitalio import DigitalInOut, Direction, Pull

mosi_pin = board.GP11
clk_pin = board.GP10
reset_pin = board.GP17
cs_pin = board.GP18
dc_pin = board.GP16
btn_pin = board.GP13

btn = DigitalInOut(btn_pin)
btn.direction = Direction.INPUT
btn.pull = Pull.UP

displayio.release_displays()

spi = busio.SPI(clock=clk_pin, MOSI=mosi_pin)

display_bus = displayio.FourWire(spi, command=dc_pin, chip_select=cs_pin, reset=reset_pin)

display = ST7735R(display_bus, width=132, height=132, bgr = True, rotation=90)

splash = displayio.Group()
display.show(splash)

# Settings
font_file = "fonts/Ubuntu-Bold-38.bdf"
x_offset = 6
y_offset = -10
multiplier = 3
black_text_on_white_bg = True

black_color = 0x000000
white_color = 0xFFFFFF

font = bitmap_font.load_font(font_file)
font_palette = displayio.Palette(2)

if black_text_on_white_bg:
    font_palette[0] = white_color
    font_palette[1] = black_color
else:
    font_palette[0] = black_color
    font_palette[1] = white_color



def generate_bitmap(text, x_offset, y_offset, multiplier=1):
    bitmap = displayio.Bitmap(display.width, display.height, 2)

    _, height, _, dy = font.get_bounding_box()
    for y in range(height):
        pixels = []
        for c in text:
            glyph = font.get_glyph(ord(c))
            if not glyph:
                continue
            glyph_y = y + (glyph.height - (height + dy)) + glyph.dy

            if 0 <= glyph_y < glyph.height:
                for i in range(glyph.width):
                    value = glyph.bitmap[i, glyph_y]
                    pixel = 0
                    if value > 0:
                        pixel = 1
                    pixels.append(pixel)
            else:
                # empty section for this glyph
                for i in range(glyph.width):
                    pixels.append(0)

            # one space between glyph
            pixels.append(0)

        if pixels:
            for x, pixel in enumerate(pixels):
                for i in range(multiplier):
                    x_coord = multiplier*x + i + x_offset

                    if x_coord >= 0 and x_coord < display.width:
                        for j in range(multiplier):

                            y_coord = multiplier*y + j + y_offset

                            if y_coord >= 0 and y_coord < display.height:
                                bitmap[x_coord, y_coord] = pixel

    return bitmap

def pad_number(digit):
    if digit < 10:
        return '0' + str(digit)
    else:
        return str(digit)


splash.append(
    displayio.TileGrid(
        generate_bitmap(pad_number(0), x_offset, y_offset, multiplier),
        pixel_shader=font_palette
    )
)

count = 0

btn_last_state = btn.value

while True:

    # change displayed number on button press
    btn_curr_state = btn.value

    if (btn_curr_state != btn_last_state):
        if not btn_curr_state:
            sleep(0.25)

            # if still pressed (long press), reset count
            if not btn.value:
                count = 0

            # otherwise increment count
            else:
                count += 1

            # update display
            splash[0] = displayio.TileGrid(
                generate_bitmap(pad_number(count), x_offset, y_offset, multiplier),
                pixel_shader=font_palette
            )

        btn_last_state = btn_curr_state