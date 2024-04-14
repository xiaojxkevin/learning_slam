from PIL import Image
import os

def png_to_gif(png_files, output_file):
    images = []
    for file in png_files:
        try:
            img = Image.open(file)
            images.append(img)
        except Exception as e:
            print(f"Error processing {file}: {e}")
    
    # Save images as gif
    images[0].save(output_file, save_all=True, append_images=images[1:], loop=0, duration=duration)
    
    print(f"GIF file saved as {output_file}")

def main():
    png_files = [os.path.join(file_dir, file) for file in os.listdir(file_dir) if file.endswith('.png')]
    png_files = sorted(png_files)
    if not png_files:
        print("No PNG files found in the current directory.")
        return
    
    output_file = "output.gif"
    png_to_gif(png_files, output_file)

if __name__ == "__main__":
    file_dir = "/home/jinxi/codes/learning_slam/mcl_slam/results/rgb"
    duration = 150
    main()
