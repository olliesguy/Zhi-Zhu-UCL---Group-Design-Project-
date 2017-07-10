#!/usr/bin/env python

# Serial communication
import serial

# Image processing/Array manipulation
from PIL import Image
import numpy as np
from scipy import ndimage as ndimage

# Display of image and drawing progress
import pygame

# GUI interface
import tkinter
from tkinter import filedialog

# math
import math

# Physical canvas dimensions - should probably read these from Arduino.  Units are 0.1mm.
canvasorigin = [2000, 2700]
canvassize = [4000, 3500]
canvascenter = [canvasorigin[0] + canvassize[0]/2, canvasorigin[1] + canvassize[1]/2]

# Maximum pixels allowed per side in the image to plot, we assume all pixels are square.
# Eventually we want to make this a user input to allow for greater image detail.
MAXIMSIZE = 64

CMD_UNKNOWN = 0
CMD_OK = 1
CMD_CONSOLE = 2

PEN_UP = 0
PEN_DOWN = 1

DIR_CW = 0
DIR_CCW = 1

IMG_SCALAR_SHADED = 0
IMG_SCALAR_CROSSHATCH = 1
IMG_VECTOR_SVG = 2

OS_WINDOWS = 0
OS_RASPBIAN = 1
# Set this to the current machine OS.
CURRENT_OS = OS_WINDOWS

# Helper function - returns rounded interger
def rint(x):
    return int(round(x))


# Returns string minus trailing "\r\n"
#def stripreturn(str):
    #return str.rstrip("\r\n")


# Helper function - returns artcangent angle between 0...2Pi. Taken from:
# https://www.marginallyclever.com/blog/2014/03/how-to-improve-the-2-axis-cnc-gcode-interpreter-to-understand-arcs/
def atan3(dy, dx):
    a = math.atan2(dy, dx)
    if a < 0:
        a = a + 2*np.pi
    return a


# Process commands received from Arduino.
def processdata(line):
    # Line comes in binary - convert to string type.
    line = line.decode('ascii')
    if line[0] == '#':
        return CMD_CONSOLE
    elif line.find("OK") == 0:
        return CMD_OK
    else:
        return CMD_UNKNOWN


# Return instructions for stepping through a line.
def parse_line(args, penmode):
    ins = None
    dic = parse_args(args)
    if ('X' in dic) and ('Y' in dic):
        mode = 'M' if penmode == PEN_UP else 'L'
        # Note we multipy Y variable by -1 because our Y-axis points down.
        ins = [mode, dic['X'], -1*dic['Y']]
        #print(ins)
    return ins


# Return instructions for stepping through an arc. Much of this code taken from:
# https://www.marginallyclever.com/blog/2014/03/how-to-improve-the-2-axis-cnc-gcode-interpreter-to-understand-arcs/
def parse_arc(args, direction, lastpos):
    instr = []

    dic = parse_args(args)
    dx = dic.get('I', 0)
    dy = dic.get('J', 0)
    arc_center = [lastpos[0] + dx, lastpos[1] + dy]
    arc_end = [dic.get('X'), dic.get('Y')]
    radius = math.sqrt(dx**2 + dy**2)
    theta_i = atan3(-dy, -dx)
    theta_f = atan3(arc_end[1] - arc_center[1], arc_end[0] - arc_center[0])
    sweep = theta_f - theta_i

    if (direction == DIR_CW and sweep < 0):
        theta_f = theta_f + 2*PI
    elif (direction == DIR_CCW and sweep > 0):
        theta_i = theta_i + 2*PI

    # Arc length in units of 0.1mm
    arc_length = abs(sweep)*radius
    #print("arc length", arc_length)

    # Specify how long each arc segment will be
    CM_PER_SEGMENT = .025
    num_segments = int(arc_length/CM_PER_SEGMENT)

    # Interpolate around the arc
    for i in range(num_segments):
        fraction = i/num_segments
        theta_mid = sweep*fraction + theta_i
        x_i = arc_center[0] + math.cos(theta_mid)*radius
        y_i = arc_center[1] + math.sin(theta_mid)*radius
        #print(['L', x_i, y_i])
        instr.append(['L', x_i, y_i])

    #print(['L', arc_end[0], arc_end[1]])
    instr.append(['L', arc_end[0], arc_end[1]])

    return instr


# Returns a dictionary of argument-value pairs.
# Copied from Jonathan Win's gcode viewer at github.com/jonathanwin/yagv
def parse_args(argstr):
    dic = {}
    if argstr:
        bits = argstr.split()
        for bit in bits:
            letter = bit[0]
            coord = float(bit[1:])
            dic[letter] = coord
    return dic


# Convert a line of gcode to a wall plotter instruction.
# Currently handles G0 (move), G1 (line), G2 (CW arc), G3 (CCW arc)
# Bits of code copied from Jonathan Win's gcode viewer (yagv) - see github.com/jonathanwin/yagv
def parse_gcode(line, instructions, curpos):

    # Remove any trailing spaces.
    line = line.rstrip()

    # Strip any comments.
    index = line.find(';')
    if index > 0:
        line = line[:index]

    # Strip any items in parenthesis.
    index = line.find('(')
    if index > 0:
        line = line[:index]

    # Get command and args
    command = line.split(None, 1)
    code = None
    args = None
    if len(command) > 0:
        code = command[0]
    if len(command) > 1:
        args = command[1]

    # Turn command into instrutions.
    if code and code[0] == "G":
        #print(line)
        num = int(float(code[1:]))
        #print("num = ", num)
        if (num == 0) or (num == 1):
            penmode = PEN_UP if num == 0 else PEN_DOWN
            ins = parse_line(args, penmode)
            if ins:
                curpos = ins[1:]
                instructions.append(ins)

        elif (num == 2) or (num == 3):
            direction = DIR_CW if num == 2 else DIR_CCW
            instr = parse_arc(args, direction, curpos)
            if instr:
                n = len(instr)
                curpos = instr[n-1][1:]
                instructions = instructions + instr

        else:
            print("Command ", line, " doesn't translate to known instruction")
    else:
        print("Command ", code, " not recognized")

    return curpos


# Create instructions list from gcode file.
def vectorimage(filename):

    instructions = []
    curpos = [0,0]
    with open(filename) as f:
        for line in f:
            curpos = parse_gcode(line, instructions, curpos)

    # Scale the instructions to the canvas size.
    x_values = [float(x) for x in np.array(instructions)[:,1]]
    y_values = [float(y) for y in np.array(instructions)[:,2]]

    max_x = max(x_values)
    min_x = min(x_values)
    max_y = max(y_values)
    min_y = min(y_values)

    print("max x = ", max_x)
    print("max y = ", max_y)
    print("min x = ", min_x)
    print("min y = ", min_y)

    x_size = max_x - min_x
    y_size = max_y - min_y

    scalefactor = 1.0
    if (canvassize[0]/x_size > canvassize[1]/y_size):
        scalefactor = canvassize[1]/y_size
    else:
        scalefactor = canvassize[0]/x_size

    for ins in instructions:
        ins[1] = rint((ins[1] - min_x)*scalefactor + canvasorigin[0])
        ins[2] = rint((ins[2] - min_y)*scalefactor + canvasorigin[1])

    # Hopefully this will cause the drawing to end with pen lifted.
    instructions.append(['M', 0, 0])

    #print(instructions)
    return instructions


# Converts a pygame surface to grayscale values.
def grayscale_surface(surf):
    width, height = surf.get_size()
    for x in range(width):
        for y in range(height):
            red, green, blue, alpha = surf.get_at((x, y))
            L = 0.3 * red + 0.59 * green + 0.11 * blue
            gs_color = (L, L, L, alpha)
            surf.set_at((x, y), gs_color)
    return surf


# Set any transparent pixels to white in a PIL image.
def transparent_to_white(im):
    if (im.mode == 'RGBA'):
        bottom_im = Image.new("RGBA", (im.size[0], im.size[1]), "white")
        r,g,b,a = im.split()
        im = Image.merge("RGB", (r, g, b))
        mask = Image.merge("L", (a, ))
        bottom_im.paste(im, (0,0), mask)
        return bottom_im
    else:
        return im


# Inverts the colors in a Surface Array - by converting to image.
def invert_surface(sur):
    inv = pygame.Surface(sur.get_rect().size, pygame.SRCALPHA)
    inv.fill((255,255,255,255))
    inv.blit(sur, (0,0), None, pygame.BLEND_RGB_SUB)
    return inv


# Takes a PIL image, then displays it with pygame and returns the pygame surface.
def showimg(img):
    window_size = (640, 480)
    if (CURRENT_OS == OS_RASPBIAN):
        window_size = (480, 320)
    img_size = img.size

    # Maximize the size of the displayed image.
    scalefactor = min(window_size[0]/img_size[0], window_size[1]/img_size[1])

    # Create the window for the display.
    screen = pygame.display.set_mode(window_size)

    # Convert PIL image to Pygame Surface.
    # Pygame doesn't handle 8-bit images well, so convert to RBG if it is Grayscale.
    if (img.mode == 'L'):
        img = img.convert('RGB')
    imstring = img.tobytes()
    sur = pygame.image.fromstring(imstring, img.size, img.mode)
    # Convert to grayscale
    sur = grayscale_surface(sur)

    # Scale the image up to an appropriate size for display.
    (w, h) = sur.get_size()
    factor = int(scalefactor)
    #print("factor = ", factor)
    sur = pygame.transform.scale(sur, (w*factor, h*factor))

    screen.blit(sur, (0, 0))
    pygame.display.flip()


# Returns instructions list for a crosshatch shaded image.
# Scale is the  number of mm per 'pixel' and imarray is a numpy array containing the greyscale image.
def make_crosshatch_image(scalefactor, imarray):
    # Array to be returned
    instructions = []

    # Scale the values in imarray from 0 to MAXINTENSITY
    MAXINTENSITY = 255
    minval = imarray.min()
    maxval = imarray.max()
    intensity_scale = MAXINTENSITY/(maxval - minval)
    imarray = intensity_scale*(imarray - minval)

    # The center of the image (in units of 0.1mm) is the point around which it is rotated.
    centerx = imarray.shape[1]*scalefactor/2.0
    centery = imarray.shape[0]*scalefactor/2.0

    # startx, starty are the upper left corner of the image
    startx = canvascenter[0] - centerx
    starty = canvascenter[1] - centery

    # The number of crosshatch layers to lay down.
    NLAYERS = 6
    angle_interval = np.pi/NLAYERS

    # Start with one line per image row.
    print("Computing ", NLAYERS, " crosshatch arrays")
    FILLER_PIXEL_VAL = MAXINTENSITY + 1

    for layer in range(NLAYERS):
        angle = layer*angle_interval
        print("angle = ", rint(math.degrees(angle)))
        # Ndimage rotate function rotates in the opposite direction of our coordinate system,
        # so the angle for our rotation has the same sign as the angle passed to ndimage.rotate
        c = math.cos(angle)
        s = math.sin(angle)

        # Rotate image into bigger array.  Extra pixels generated get set to FILLER_PIXEL_VAL
        rot_array = ndimage.rotate(imarray, math.degrees(angle), mode='constant', cval=FILLER_PIXEL_VAL)
        threshold = MAXINTENSITY*(NLAYERS - layer - 0.25)/NLAYERS

        # Lower intensity pixels get drawn darker (more lines).  Create an array of booleans,
        # which are true if they are part of the line and false if not.
        line_array = rot_array < threshold

        # Center of the rotation - we'll need to rotate the line coordinates back around this point.
        center_pixel = [line_array.shape[1]/2.0, line_array.shape[0]/2.0]

        direction = 1
        line_start = (0.0, 0.0)
        line_end = (0.0, 0.0)
        ins = []
        for j, row in enumerate(line_array):
            line_started = False
            # Alternate directions in which we traverse the rows
            for i, pixel in (enumerate(row) if (direction > 0) else reversed(list(enumerate(row)))):
                if pixel and not line_started:
                    line_started = True
                    x = scalefactor*(i - center_pixel[0])
                    y = scalefactor*(j - center_pixel[1])
                    if (direction < 0):
                        x = x + scalefactor
                    xp = c*x - s*y
                    yp = s*x + c*y
                    line_start = (canvascenter[0] + xp, canvascenter[1] + yp)
                elif line_started and not pixel:
                    line_started = False
                    x = scalefactor*(i - center_pixel[0])
                    y = scalefactor*(j - center_pixel[1])
                    if (direction < 0):
                        x = x + scalefactor
                    xp = c*x - s*y
                    yp = s*x + c*y
                    line_end = (canvascenter[0] + xp, canvascenter[1] + yp)
                    # If line is longer than 1 cm (100 x 0.1mm), then subdivide it
                    d = distance(line_start, line_end)
                    nseg = max(rint(d/100), 1)
                    ins = draw_divided_line(line_start, line_end, nseg)
                    #check_out_of_bounds_list(ins)
                    instructions = instructions + ins

            # At the end of each row, finish any lines which have been started
            if line_started:
                line_started = False
                x = direction*scalefactor*center_pixel[0]
                y = scalefactor*(j - center_pixel[1])
                xp = c*x - s*y
                yp = s*x + c*y
                line_end = (canvascenter[0] + xp, canvascenter[1] + yp)
                d = distance(line_start, line_end)
                nseg = max(rint(d/100), 1)
                ins = draw_divided_line(line_start, line_end, nseg)
                #check_out_of_bounds_list(ins)
                instructions = instructions + ins

            direction = direction*-1

    # Lift the pen up and get it out of the way after drawing
    instructions.append(['M', rint(canvasorigin[0]), rint(canvasorigin[1])])
    return instructions


# im is a 2D numpy array containing image values.  Takes a NumPy array
def make_shaded_image(scalefactor, imarray):
    # Maximum intensity of image pixel
    MAXINTENSITY = math.ceil(imarray.max())

    imsize = imarray.shape
    imwidth = imsize[1]
    imheight = imsize[0]
    startx = canvascenter[0] - imwidth*scalefactor/2
    starty = canvascenter[1] - imheight*scalefactor/2
    lastx = startx
    lasty = starty

    # First instruction sends pen to starting point
    instructions = [['M', rint(startx), rint(starty)]]

    # Maximum number of strokes per pixel.  Calculated to make stroke size a minumum of around 1mm
    MAXSTROKES = int(canvassize[0]/(MAXIMSIZE*10))

    # Try larger MAXSTROKES to see what happens (usually = 6)
    #MAXSTROKES = 10

    jitterheight = 2*scalefactor/3 # Height of pen strokes
    direction = 1
    # Step through rows of the image
    for i, row in enumerate(imarray):
        # Step through pixels in the row
        for j, pixel in (enumerate(row) if (direction > 0) else reversed(list(enumerate(row)))):
            # Invert image intensity (white = low / dark = high)
            intensity = (MAXINTENSITY - pixel)/MAXINTENSITY
            jitter = rint(MAXSTROKES*intensity)
            if jitter == 0:
                lastx = lastx + direction*scalefactor
                instructions.append(['L', rint(lastx), rint(lasty)])
            else:
                jitterwidth = scalefactor/jitter
                dx = jitterwidth/2
                for k in range (0, jitter):
                    instructions.append(['L', rint(lastx), rint(lasty + jitterheight)])
                    instructions.append(['L', rint(lastx + direction*dx), rint(lasty + jitterheight)])
                    instructions.append(['L', rint(lastx + direction*dx), rint(lasty)])
                    instructions.append(['L', rint(lastx + direction*2*dx), rint(lasty)])
                    lastx = lastx + direction*jitterwidth
                    # Next pixel
        lasty = starty + (i+1)*scalefactor
        direction = -1*direction
        # Lift the pen carriage up to get to the new line.
        instructions.append(['M', rint(lastx), rint(lasty)])

    # Get the pen carriage up and out of the way when done.
    instructions.append(['M', canvasorigin[0], canvasorigin[1]])

    #print(instructions[0:50])
    return instructions


# Create instructions list from image.
def scalarimage(imfile, shadetype):
    # Import and manipulate the picture.
    im = Image.open(imfile)

    # See if aspect ratio of image is greater or less than canvas aspect ratio.
    # Then scalefactor up the image to MAXIMSIZE pixels on the largest side.
    # Add a slight margin for error to make sure we don't exceed boundaries.
    scalefactor = 1
    margin = 200
    if im.size[1]/im.size[0] > canvassize[1]/canvassize[0]:
        scalefactor = (canvassize[1] - margin)/MAXIMSIZE
    else:
        scalefactor = (canvassize[0] - margin)/MAXIMSIZE
    im.thumbnail((MAXIMSIZE, MAXIMSIZE), Image.ANTIALIAS)

    # Get new size
    imsize = im.size
    print("imsize = ", imsize)
    print("scalefactor = ", scalefactor)

    # If the image has an alpha channel set all transparent pixels to white.
    im = transparent_to_white(im)

    # Show the image on the screen.
    showimg(im)

    # Convert image to array.
    imarray = np.array(im.convert('L'))


    instructions = []
    if (shadetype == IMG_SCALAR_SHADED):
        instructions = make_shaded_image(scalefactor, imarray)
    elif (shadetype == IMG_SCALAR_CROSSHATCH):
        instructions = make_crosshatch_image(scalefactor, imarray)

    return instructions


def distance(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


# Creates instructions to draw a line composed of n increments from start to end.
def draw_divided_line(firstpt, lastpt, n):
    if (n <= 0):
        print("Can't divide line into 0/- segments")
        return []

    instructions = [['M', rint(firstpt[0]), rint(firstpt[1])]]

    xinc = (lastpt[0] - firstpt[0])/n
    yinc = (lastpt[1] - firstpt[1])/n

    for i in range(1, n+1):
        instructions.append(['L', rint(firstpt[0] + i*xinc), rint(firstpt[1] + i*yinc)])

    return instructions


def check_out_of_bounds(ins):
    if ((ins[1] < canvasorigin[0]) or
        (ins[1] > canvasorigin[0] + canvassize[0]) or
        (ins[2] < canvasorigin[1]) or
        (ins[2] > canvasorigin[1] + canvassize[1])):
        return True
    else:
        return False


# Checks a list of instructions to see if any of the coordinates are out of bounds.
def check_out_of_bounds_list(instructions):
    all_in_bounds = True
    for ins in instructions:
        if check_out_of_bounds(ins):
            print("Out of bounds:", ins)
            all_in_bounds = False

    return all_in_bounds


# Draw a pattern to test the printer.
def draw_test_pattern():
    instructions = []

    margin = 100
    co = [canvasorigin[0] + margin, canvasorigin[1] + margin]
    cs = [canvassize[0] - margin, canvassize[1] - margin]

    # Draw a square around the perimeter of the canvas.
    instructions.append(['M', rint(co[0]), rint(co[1])])
    instructions.append(['L', rint(co[0] + cs[0]), rint(co[1])])
    instructions.append(['L', rint(co[0] + cs[0]), rint(co[1] + cs[1])])
    instructions.append(['L', rint(co[0]), rint(co[1] + cs[1])])
    instructions.append(['L', rint(co[0]), rint(co[1])])

    # Draw a big 'X' across the square, then return to origin.
    instructions.append(['L', rint(co[0] + cs[0]), rint(co[1] + cs[1])])
    instructions.append(['M', rint(co[0] + cs[0]), rint(co[1])])
    instructions.append(['L', rint(co[0]), rint(co[1] + cs[1])])

    # Now draw the same lines broken into smaller steps.  Should be straighter when drawn in segments.
    ninc = 40
    instructions = instructions + draw_divided_line([co[0], co[1]], [co[0] + cs[0], co[1]], ninc)
    instructions = instructions + draw_divided_line([co[0] + cs[0], co[1]], [co[0] + cs[0], co[1] + cs[1]], ninc)
    instructions = instructions + draw_divided_line([co[0] + cs[0], co[1] + cs[1]], [cs[1], co[1] + cs[1]], ninc)
    instructions = instructions + draw_divided_line([co[0], co[1] + cs[1]], [co[0], co[1]], ninc)
    instructions = instructions + draw_divided_line([co[0], co[1]], [co[0] + cs[0], co[1] + cs[1]], ninc)
    instructions = instructions + draw_divided_line([co[0] + cs[0], co[1]], [co[0], co[1] + cs[1]], ninc)
    instructions.append(['M', rint(co[0]), rint(co[1])])

    #check_out_of_bounds(instructions)
    return instructions


def main():
    # Show askopenfilename dialog without the Tkinter window.
    root = tkinter.Tk()
    root.withdraw()
    dirname = ""
    if (CURRENT_OS == OS_WINDOWS):
        dirname = "C:/Users/ollie/"
    #elif (CURRENT_OS == OS_RASPBIAN):
    #    dirname = "/home/pi/WallPlotterImages"
    filename = filedialog.askopenfilename(title="Choose a Data File",
                                          initialdir=dirname,
                                          filetypes=[('all files', '.*'), ('jpg files', '.jpg')])
    print(filename)

    print("Enter graphics type:")
    print("(1) Image file shading")
    print("(2) Image file crosshatch")
    print("(3) GCode")
    print("(4) Test Pattern")
    option = int(input("Select number of desired option "))

    instructions = []
    if option == 1:
        instructions = scalarimage(filename, IMG_SCALAR_SHADED)
    elif option == 2:
        instructions = scalarimage(filename, IMG_SCALAR_CROSSHATCH)
    elif option == 3:
        instructions = vectorimage(filename)
    elif option == 4:
        instructions = draw_test_pattern()

    # Open serial connection to Arduino
    #port_name = ""
    #if (CURRENT_OS == OS_WINDOWS):
    #    port_name = "COM3"
    #elif(CURRENT_OS == OS_RASPBIAN):
    #    port_name = "/dev/ttyUSB0"
    arduino = serial.Serial("COM3", 9600)
    print(arduino.name)

    # Wait for connection to establish
    connected = False
    while not connected:
        serin = arduino.read()
        connected = True

    # Send directions to the Arduino and listen for feedback
    while True:
        line = arduino.readline()
        if line:
            cmd = processdata(line)
            if (cmd == CMD_CONSOLE):
                print(line)
            elif (cmd == CMD_OK):
                if len(instructions) == 0:
                    print("finished")
                else:
                    # Obtain and remove the first instruction from the list
                    inst = instructions.pop(0)
                    buf = "%c %d %d;" % (inst[0], inst[1], inst[2])
                    #print("buf = " + buf)
                    arduino.write(buf.encode('ascii'))
            else:
                print("Command received but not understood:")
                print(line)

main()
