import pandas as pd
from PIL import Image

RESULT_NAME = "Result.xlsx"

dataFrame = pd.read_excel(RESULT_NAME)

#removing Unneeded Variables 
dataFrame.pop("Var4")
dataFrame.pop("Var5")
dataFrame.pop("Var6")
dataFrame.pop("Var7")
dataFrame.pop("Var8")
dataFrame.pop("Var9")

readings = pd.DataFrame(dataFrame[1:])

print(readings)

im = Image.new(mode="RGB", size=(200,200), color='red')

im.save("resultingImg.png")

