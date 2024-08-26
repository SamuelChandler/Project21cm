import tkinter 
from tkinter import filedialog
from PIL import Image

tkinter.Tk().withdraw() #prevents window from opening 

IMG_FILENAME = "./resultingImg.png"

dest_folder = filedialog.askdirectory()

image = Image.open(IMG_FILENAME)

PathToImage = dest_folder+"/ImageResult.png"

print("Saving to "+PathToImage)

image.save(PathToImage)

