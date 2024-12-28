import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import filedialog
from PIL import Image, ImageTk

class log_viewer():
    def __init__(self):
        self.data = None

        self.root = tk.Tk()
        self.root.geometry("1000x900")
        self.root.title("Log viewer")

        button = tk.Button(self.root, text="Select log file", command=self.select_file)
        button.pack(side=tk.BOTTOM)

        self.root.mainloop()
        pass

    def select_file(self):
        # ファイル選択ダイアログを開く
        file_path = filedialog.askopenfilename()
        
        self.get_csv_data(file_path)

    def get_csv_data(self, file_path):
        self.data = pd.read_csv(file_path)
        columns = self.data.columns
        #check data using python dictionary list
        arrays_dict = {col: self.data[col].astype(float).tolist() for col in columns}
        keys = arrays_dict.keys()
        keys_list = list(keys)
        self.display_graph()
    
    def plot_graph(self):
        fig, axes = plt.subplots(5, 2, figsize=(9, 12))
        axes = axes.flatten()

        for i, column in enumerate(self.data.columns[1:]):
            sns.lineplot(x='time', y=column, data=self.data, ax=axes[i])
            axes[i].set_title(column)
            axes[i].set_xlabel('Time')
            axes[i].set_ylabel('Value')

        plt.tight_layout()
        return fig
    
    def display_graph(self):
        fig = self.plot_graph()
        canvas = FigureCanvasTkAgg(fig, master=self.root)
        canvas.draw()
        canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

viewer = log_viewer()