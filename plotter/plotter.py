import pandas as pd
import plotly.express as px

df = pd.read_csv("output.csv", header=None, names=["X", "Y", "Z"])

fig = px.scatter_3d(
    df,
    x="X",
    y="Y",
    z="Z",
    color="Z",
    title="Interactice 3D Visualization",
)

html_file = "plot.html"
fig.write_html(html_file)
