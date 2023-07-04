import tkinter as tk
from PIL import Image, ImageTk
import pyglet
import threading

class Player:
    def __init__(self, gif_file=None, audio_file=None):
        self.gif_file = gif_file
        self.audio_file = audio_file
        self.window = tk.Tk()
        self.window.geometry("400x400")
        self.gif_label = None
        self.gif_photo = None  # Reference to PhotoImage object
        self.music = None
        self.thread = None

    def play(self):
        if self.gif_file is not None:
            # Load and display the GIF image
            gif_image = Image.open(self.gif_file)
            gif_image = gif_image.resize((400, 400), Image.ANTIALIAS)
            self.gif_photo = ImageTk.PhotoImage(gif_image)
            self.gif_label = tk.Label(self.window, image=self.gif_photo)
            self.gif_label.pack()

        if self.audio_file is not None:
            # Load and play the audio file using pyglet
            music = pyglet.resource.media(self.audio_file)
            self.music = pyglet.media.Player()
            self.music.queue(music)
            self.music.play()

        self.window.mainloop()

    def close(self):
        self.window.destroy()
        if self.music is not None:
            self.music.pause()

    def start_playback(self):
        self.thread = threading.Thread(target=self.play)
        self.thread.start()

    def update_files(self, gif_file=None, audio_file=None):
        self.close()  # Close current playback

        # Update with new files
        self.gif_file = gif_file
        self.audio_file = audio_file

        # Start new playback in a separate thread
        self.start_playback()

player = None  # Global variable for the player object

def play_gif_and_audio(gif_file=None, audio_file=None):
    global player

    if gif_file is None and audio_file is None:
        return "No files provided. Nothing to play."

    player = Player(gif_file, audio_file)
    player.start_playback()

    return "Playback started."

import tkinter as tk
from PIL import Image, ImageTk
import pyglet
import threading
import time

class Player:
    def __init__(self):
        self.window = tk.Tk()
        self.window.geometry("400x400")
        self.gif_label = None
        self.music = None
        self.gif_file = None
        self.audio_file = None
        self.stop_event = threading.Event()
        self.thread = None

    def play(self, gif_file=None, audio_file=None):
        if gif_file is not None:
            self.gif_file = gif_file
            self.stop_event.clear()
            self.update_gif()

        if audio_file is not None:
            self.audio_file = audio_file
            self.stop_event.clear()
            self.update_audio()

        self.window.mainloop()

    def update_gif(self):
        if self.gif_label is not None:
            self.gif_label.destroy()

        if self.gif_file is not None:
            gif_image = Image.open(self.gif_file)
            gif_image = gif_image.resize((400, 400), Image.ANTIALIAS)
            gif_photo = ImageTk.PhotoImage(gif_image)
            self.gif_label = tk.Label(self.window, image=gif_photo)
            self.gif_label.image = gif_photo
            self.gif_label.pack()

    def update_audio(self):
        if self.music is not None:
            self.music.pause()

        if self.audio_file is not None:
            music = pyglet.resource.media(self.audio_file)
            self.music = pyglet.media.Player()
            self.music.queue(music)
            self.music.play()

    def close(self):
        self.stop_event.set()
        self.window.destroy()
        if self.music is not None:
            self.music.pause()

    def playback_loop(self):
        while not self.stop_event.is_set():
            self.window.update()

        self.window.destroy()

def play_gif_and_audio(gif_file=None, audio_file=None):
    player = Player()
    player.play(gif_file, audio_file)

    return "Playback complete."

# Example usage:
player = Player()
player_thread = threading.Thread(target=player.playback_loop)
player_thread.start()

# Play initial files
player.play("example1.gif", "example1.mp3")

# Wait for 12 seconds
time.sleep(5)

# Update files
player.play("example2.gif", "example2.mp3")

# Stop playback and close the player
player.close()
player_thread.join()

print("Closed by the supervisor.")
def update_files(gif_file=None, audio_file=None):
    if player is None:
        return "No player object exists. Please start playback first."

    player.update_files(gif_file, audio_file)
    return "Files updated."

# Example usage:
status = play_gif_and_audio("example1.gif", "example1.mp3")
print(status)

# Example usage to update files:
update_status = update_files("example2.gif", "example2.mp3")
print(update_status)
