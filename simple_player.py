#!/usr/bin/env python3
import socket
import subprocess
import threading
import os

PORT = 8888

# Your existing 4 drum sound files
SOUND_FILES = {
    "snare": "/Users/keerath/Downloads/drum-snare.wav",
    "hihat": "/Users/keerath/Downloads/hi-hat1.wav", 
    "crash": "/Users/keerath/Downloads/crash-drum_F_major.wav",
    "tom": "/Users/keerath/Downloads/tom-drum.wav"
}

def get_volume_for_velocity(velocity):
    """Convert velocity to volume level for afplay"""
    volume_map = {
        "soft": 0.4,
        "medium": 0.7,
        "hard": 0.9,
        "very_hard": 1.0  # Maximum volume
    }
    return volume_map.get(velocity, 0.7)  # Default to medium

def get_velocity_emoji(velocity):
    """Get emoji for velocity"""
    emoji_map = {
        "soft": "🔹",
        "medium": "🔸", 
        "hard": "🔥",
        "very_hard": "💥"
    }
    return emoji_map.get(velocity, "🔸")

def play_drum(drum_name, velocity="medium"):
    """Play drum sound with specified velocity (just volume adjustment)"""
    
    sound_file = SOUND_FILES.get(drum_name.lower())
    
    if sound_file and os.path.exists(sound_file):
        volume = get_volume_for_velocity(velocity)
        velocity_emoji = get_velocity_emoji(velocity)
        
        print(f"🥁 {velocity_emoji} Playing {drum_name.upper()} ({velocity}, vol: {volume:.1f})")
        
        # Use afplay with volume control
        subprocess.run(['afplay', '-v', str(volume), sound_file])
    else:
        # Fallback to system sounds
        velocity_emoji = get_velocity_emoji(velocity)
        print(f"🔔 {velocity_emoji} {drum_name.upper()} ({velocity}) - system sound")
        
        # Different system sounds for different velocities
        if velocity in ["hard", "very_hard"]:
            subprocess.run(['afplay', '/System/Library/Sounds/Blow.aiff'])
        elif velocity == "soft":
            subprocess.run(['afplay', '/System/Library/Sounds/Pop.aiff'])
        else:
            subprocess.run(['afplay', '/System/Library/Sounds/Ping.aiff'])

def handle_drum_command(command):
    """Parse and handle drum commands in format 'drum:velocity' or just 'drum'"""
    command = command.strip().lower()
    print(f"📨 Received: {command}")
    
    # Parse command - handle both "drum:velocity" and "drum" formats
    if ':' in command:
        parts = command.split(':')
        if len(parts) == 2:
            drum_name, velocity = parts
            drum_name = drum_name.strip()
            velocity = velocity.strip()
        else:
            print(f"❓ Invalid command format: {command}")
            return
    else:
        # Old format - just drum name
        drum_name = command.strip()
        velocity = "medium"
    
    # Validate drum name
    if drum_name in SOUND_FILES:
        play_drum(drum_name, velocity)
    else:
        print(f"❓ Unknown drum: {drum_name}")

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', PORT))
    
    print("🎧 Air Drumming Audio Player Ready!")
    print(f"📡 Listening on port {PORT}")
    print("🥁 Available drums: snare, hihat, crash, tom")
    print("🎚️  Velocities: soft (40%), medium (70%), hard (90%), very_hard (100%)")
    print("📂 Sound files:")
    
    for drum, file_path in SOUND_FILES.items():
        status = "✅" if os.path.exists(file_path) else "❌"
        print(f"   • {drum.upper()}: {status} {os.path.basename(file_path)}")
    
    print("🎵 Waiting for drumming...\n")
    
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            command = data.decode('utf-8')
            
            # Handle in separate thread for responsiveness
            thread = threading.Thread(target=handle_drum_command, args=(command,))
            thread.daemon = True
            thread.start()
            
        except KeyboardInterrupt:
            print("\n👋 Drumming session ended!")
            break
        except Exception as e:
            print(f"❌ Error: {e}")
    
    sock.close()

if __name__ == "__main__":
    main()