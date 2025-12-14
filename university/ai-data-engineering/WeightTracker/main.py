import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import sqlite3
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import numpy as np
# set the style of the plots
sns.set(rc={'axes.facecolor':'#6c7c96', 'figure.facecolor':'#c29380'})


class DietTracker:
    def __init__(self):
        # Create the main window with a flexible layout
        self.root = tk.Tk()
        self.root.resizable(True, True)
        self.root.title("Weight Tracker")
        self.root.geometry("500x500")
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)

        # Create the main frame
        self.main_frame = ttk.Frame(self.root)
        self.main_frame.pack(fill='both', expand=True)

        # Create a notebook widget to hold the tabs
        self.main_notebook  = ttk.Notebook(self.main_frame)
        self.main_notebook.pack(fill='both', expand=True)

        # Create the first tab
        self.menu = ttk.Frame(self.main_notebook)
        self.main_notebook.add(self.menu, text='Menu')

        # Create the about tab that will hold the text information about the app
        # use a bit larger font size
        about_text = """Made by: Ruşen Birben, 2023\n 
        This is a project for the Python course at Istanbul Technical University.\n
        This app is a simple weight tracker that allows you to add your weight and date to a database and view it in a table or plot."""
        self.about = ttk.Frame(self.main_notebook)
        self.about_label = tk.Label(self.about, text=about_text, font=("Arial", 14))
        self.about_label.pack()
        self.main_notebook.add(self.about, text='About')


        # Create database connection and table
        self.conn = sqlite3.connect("weight.db")
        self.c = self.conn.cursor()
        self.c.execute("CREATE TABLE IF NOT EXISTS weights(date TEXT, weight REAL)")

        # Create GUI widgets
        tk.Label(self.menu, text="Enter your weight (Kg):").pack()
        self.weight_entry = tk.Entry(self.menu)
        self.weight_entry.pack()

        tk.Label(self.menu, text="Enter the date (YYYY-MM-DD):").pack()
        self.date_entry = tk.Entry(self.menu)
        self.date_entry.pack()

        # make buttons with size 20x2
        tk.Button(self.menu, text="Add record", command=self.add_record , width=20, height=2).pack()

        tk.Button(self.menu, text="View table", command=self.view_table , width=20, height=2).pack()

        tk.Button(self.menu, text="View plot", command=self.view_plot , width=20, height=2).pack()

        # input a height to calculate the BMI
        tk.Label(self.menu, text="Enter your height (cm):").pack()
        self.height_entry = tk.Entry(self.menu)
        self.height_entry.pack()

        # make buttons with size 20x2
        tk.Button(self.menu, text="Add BMI to plot", command=self.calculate_bmi , width=20, height=2).pack()

    def add_record(self):
        weight = self.weight_entry.get()
        date = self.date_entry.get()

        # check if date is valid
        try:
            pd.to_datetime(date)
        except ValueError:
            messagebox.showerror("Error", "Please enter a valid date")
            return
        
        if not weight or not date:
            messagebox.showerror("Error", "Please enter both weight and date")
            return

        try:
            weight = float(weight)
        except ValueError:
            messagebox.showerror("Error", "Weight must be a number")
            return

        # if date already exists, update the record
        if self.c.execute("SELECT * FROM weights WHERE date=?", (date,)).fetchone():
            self.c.execute("UPDATE weights SET weight=? WHERE date=?", (weight, date))
            self.conn.commit()
            messagebox.showinfo("Success", "Record updated successfully")
            return
        
        self.c.execute("INSERT INTO weights VALUES (?, ?)", (date, weight))
        self.conn.commit()

        messagebox.showinfo("Success", "Record added successfully")

    def view_table(self):
        self.c.execute("SELECT * FROM weights")
        rows = self.c.fetchall()

        if not rows:
            messagebox.showwarning("Warning", "No records found")
            return

        # Create a new window to display the table
        table_window = tk.Toplevel(self.menu)
        table_window.title("Weight Records")
        table_window.geometry("400x400")

        # Create a pandas dataframe widget
        table = pd.DataFrame(rows, columns=["Date", "Weight"])

        # sort the table by date in ascending order, first convert the date column to datetime
        table["Date"] = pd.to_datetime(table["Date"])
        table.sort_values(by="Date", inplace=True)
        table.reset_index(drop=True, inplace=True)
        frame = tk.Frame(table_window)
        frame.pack(fill="both", expand=True)
        table_widget = tk.Label(frame, text=table.to_string(), justify="left")
        table_widget.pack()

    def view_plot(self):
        self.c.execute("SELECT * FROM weights")
        rows = self.c.fetchall()

        if not rows:
            messagebox.showwarning("Warning", "No records found")
            return

        # Create a pandas dataframe from the database rows
        df = pd.DataFrame(rows, columns=["Date", "Weight"])

        # sort the table by date in ascending order, first convert the date column to datetime
        df["Date"] = pd.to_datetime(df["Date"])
        df.sort_values(by="Date", inplace=True)
        df.reset_index(drop=True, inplace=True)

        # Create a plot using seaborn
        fig, ax = plt.subplots()
        if hasattr(self, "bmi"):
            label = f"Weight (BMI: {self.bmi:.2f})"
        else:
            label = "Weight"
        sns.lineplot(x="Date", y="Weight", data=df, ax=ax, color="#CE1D6A", linewidth=3, label=label)
        # add a dot to the last point in the plot
        ax.scatter(df["Date"].iloc[-1], df["Weight"].iloc[-1], color="#CE1D6A", s=80, alpha=0.8)

        ax.set_title("Weight over time", fontsize=20)
        ax.set_xlabel("Date", fontsize=16)
        ax.set_ylabel("Weight", fontsize=16)

        # add the BMIs as area plots
        # possible underweight, normal, overweight, obese values
        if hasattr(self, "bmi"):
            bmi = self.bmi
            height = self.height
            underweight_limit = 18.5 * (height / 100) ** 2
            normal_limit = 24.9 * (height / 100) ** 2
            overweight_limit = 29.9 * (height / 100) ** 2

            # add the area plots
            ax.fill_between(df["Date"], 0, underweight_limit, color="#AEDB09", alpha=0.4, label="Underweight")
            ax.fill_between(df["Date"], underweight_limit, normal_limit, color="green", alpha=0.4, label="Normal")
            ax.fill_between(df["Date"], normal_limit, overweight_limit, color="#DC9B08", alpha=0.4, label="Overweight")
            ax.fill_between(df["Date"], overweight_limit, 300, color="red", alpha=0.4, label="Obese")



        # add a legend to the lower right corner
        ax.legend(loc="lower right")
        # name the figure
        fig.canvas.manager.window.wm_title("Weight Progress Plot")
        plt.show()

    def calculate_bmi(self):
        weight = self.weight_entry.get()
        height = self.height_entry.get()

        if not weight or not height:
            messagebox.showerror("Error", "Please enter both weight and height")
            return

        try:
            weight = float(weight)
            height = float(height)
        except ValueError:
            messagebox.showerror("Error", "Weight and height must be numbers")
            return

        # calculate the BMI
        self.bmi = weight / (height / 100) ** 2
        self.height = height

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    tracker = DietTracker()
    tracker.run()
