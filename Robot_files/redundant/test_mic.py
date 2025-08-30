import speech_recognition as sr
import time

def log(message):
    """Helper function to print messages with a timestamp."""
    print(f"[{time.strftime('%H:%M:%S')}] {message}")

def run_mic_test():
    """
    A dedicated script to debug microphone issues. This version allows for
    manual threshold setting and repeated tests to find a stable configuration.
    Includes timestamps for detailed debugging.
    """
    log("--- Microphone Debug Test (v4.2 - Timestamped) ---")

    # 1. List all available microphones
    try:
        mic_names = sr.Microphone.list_microphone_names()
        if not mic_names:
            log("\n[ERROR] No microphones found.")
            return

        log("\n[INFO] Found the following microphones on your system:")
        for i, name in enumerate(mic_names):
            print(f"  - Index {i}: {name}")
    except Exception as e:
        log(f"\n[ERROR] Could not list microphones. Error: {e}")
        return

    # 2. Get user input for which microphone to test
    try:
        device_index_str = input(f"\n[{time.strftime('%H:%M:%S')}] [ACTION] Please enter the index number of your microphone (e.g., 1): ")
        device_index = int(device_index_str)
    except (ValueError, IndexError):
        log("[ERROR] Invalid input.")
        return

    log(f"\n[INFO] You selected index {device_index}: \"{mic_names[device_index]}\"")

    # --- Main Test Loop ---
    while True:
        print("\n-----------------------------------------")
        
        r = sr.Recognizer()
        
        try:
            # 3. Open the selected microphone
            log(f"[DEBUG] Attempting to open microphone at index {device_index}...")
            with sr.Microphone(device_index=device_index) as source:
                log("[DEBUG] Microphone opened successfully.")

                # 4. Calibrate for ambient noise or set manually
                manual_threshold_str = input(f"[{time.strftime('%H:%M:%S')}] [ACTION] Enter a manual energy threshold (e.g., 4000) or press Enter to auto-calibrate: ")
                
                if manual_threshold_str:
                    r.energy_threshold = int(manual_threshold_str)
                    log(f"[INFO] Manual energy threshold set to: {r.energy_threshold}")
                else:
                    log("[INFO] Please be quiet. Calibrating for ambient noise (2 seconds)...")
                    r.adjust_for_ambient_noise(source, duration=2)
                    calibrated_threshold = r.energy_threshold
                    log(f"[DEBUG] Auto-calibration complete.")
                    
                    print("\n" + "*"*10 + " CALIBRATION RESULT " + "*"*10)
                    log(f"[INFO] Auto-calibrated energy threshold is: {calibrated_threshold:.2f}")
                    log(f"[ADVICE] If recognition fails, try re-running and manually entering a lower value (e.g., {int(calibrated_threshold / 2)}).")
                    print("*"*42 + "\n")


                # 5. Listen for audio
                log("[ACTION] Please say 'testing one two three' clearly!")
                log("[INFO] Listening...")

                audio = r.listen(source, timeout=5, phrase_time_limit=5)
                log("[DEBUG] Listening complete. Audio data captured.")

                # 6. Attempt to recognize the audio
                log("[INFO] Recognizing...")
                try:
                    text = r.recognize_google(audio)
                    log("\n[SUCCESS] Google recognized:")
                    print(f'>>> "{text}" <<<')
                except sr.UnknownValueError:
                    log("\n[ERROR] Google could not understand the audio.")
                    log("[ADVICE] The energy threshold might be too high. Try a lower manual value.")
                except sr.RequestError as e:
                    log(f"\n[ERROR] Could not request results from Google; {e}")

        except sr.WaitTimeoutError:
            log("\n[FATAL ERROR] Listening timed out while waiting for a phrase to start.")
            log("[ADVICE] Your voice is not loud enough to pass the energy threshold.")
            log("         Try a lower manual threshold or check system audio settings.")
        except Exception as e:
            log(f"\n[FATAL ERROR] An unexpected error occurred: {e}")

        # Ask to test again
        if input(f"\n[{time.strftime('%H:%M:%S')}] [ACTION] Test again? (y/n): ").lower() != 'y':
            break

if __name__ == "__main__":
    run_mic_test()

