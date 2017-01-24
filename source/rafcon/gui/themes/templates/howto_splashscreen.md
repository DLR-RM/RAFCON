# How to create a splash screen from the template?

- Open the template with gimp
- First select single window mode: Windows -> Single Window Mode
  - it will save you alot of time and struggle
- Check if the layer widget is present, it should contain fice items, one grey "layer Background" and four "RAFCON_logo" layers
  - If not, press Ctrl+L
- Now go to "File" -> "Open as Layers" and select a picture of your choice
- Check if the image is placed between "Background" and "RAFCON_Logo" in the Layer widget
  - If not, drag it in the correct position
- Now select the layer with your picture in the Layer widget
  - Go to the tools widget and select the "Scale" tool
  - Click on your picture with the scale tool and fit it in the 570x320px field as it pleases you.
- Notice the small eye in the Layer widget next to a layer? 
  - Activate the visibility of a logo which serves your needs the best
- If everything is done, ignore the invisible layers and click on "File" -> "Export As" 
  - Export the picture as whatever python is able to load into a pixelbuffer
- Voila, there is your normed splash screen!
