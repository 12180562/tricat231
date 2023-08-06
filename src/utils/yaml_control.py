import yaml
import tkinter as tk
from tkinter import filedialog, messagebox

class YAMLModifier:
    def __init__(self, master):
        self.master = master
        self.master.title("YAML Modifier")

        self.load_button = tk.Button(master, text="Load YAML", command=self.load_yaml)
        self.load_button.grid(row=0, column=0, columnspan=2)

        self.entries = {}

    def load_yaml(self):
        filepath = filedialog.askopenfilename(filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")])
        if not filepath:
            return
        with open(filepath, 'r') as file:
            self.data = yaml.safe_load(file)

        self.filepath = filepath

        # Remove old entries
        for entry in self.entries.values():
            entry.grid_forget()

        self.entries = {}

        # Create new entries
        for index, (key, value) in enumerate(self.data.items(), start=1):
            tk.Label(self.master, text=key).grid(row=index, column=0)
            if isinstance(value, list):
                value = ', '.join(map(str, value))
            self.entries[key] = tk.Entry(self.master)
            self.entries[key].insert(0, value)
            self.entries[key].grid(row=index, column=1)

        self.save_button = tk.Button(self.master, text="Save", command=self.save_yaml)
        self.save_button.grid(row=index+1, column=0, columnspan=2)

    def save_yaml(self):
        for key, entry in self.entries.items():
            value = entry.get()
            try:
                self.data[key] = yaml.safe_load(value)
            except ValueError:
                messagebox.showerror("Error", "Invalid input for " + key)
        with open(self.filepath, 'w') as file:
            yaml.dump(self.data, file, Dumper=yaml.SafeDumper)

root = tk.Tk()
my_gui = YAMLModifier(root)
root.mainloop()
