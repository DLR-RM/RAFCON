
This is a guide on creating a splash screen picture using the GIMP template.

**Location**
The template file is located in `source/rafcon/gui/assets/themes/templates`. The standard place for
images to be loaded into the startup splashscreen is `source/rafcon/gui/assets/splashscreens`.

**File**
The template is a `.xcf` which is the native image format of GIMP. This guide is also using GIMP.

**Process**

1. Open the template with GIMP.
2. First select single window mode: `Windows` -> `Single Window Mode`.

  - this will save you a lot of time and struggle.

3. Check if the Layer widget is present, it should contain five items, one layer `Background` and four `RAFCON_logo` layers.

  - If not present, press `Ctrl` + `L`.

4. Now go to `File` -> `Open as Layers` and select a picture of your choice.
5. Check if the image is placed between `Background` and `RAFCON_Logo` in the Layer widget.

  - If not, drag it in the correct position.

6. Now select the layer with your picture in the Layer widget.

  - Go to the Tools widget and select the `Scale` tool.
  - Click on your picture with the `Scale` tool and fit it in the 570x320px field as it pleases you.
  - If your not satisfied with your result, try the "Move" tool and move the layer around.

7. Notice the small eye in the Layer widget next to a layer?

  - Activate the visibility of a logo which serves your needs the best.

7. If everything is done, ignore the invisible layers and click on `File` -> `Export As`

  - Export the picture as whatever python is able to load into a pixelbuffer, I'd take .jpg for pictures with lots of differently coloured areas and `.png` if you have large, single coloured chunks

9. Voila, there is your normed splash screen!
