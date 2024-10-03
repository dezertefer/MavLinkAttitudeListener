import cv2
import cv2.aruco as aruco
import argparse


# Function to generate and save an ArUco marker
def generate_aruco_marker(id, size=300, file_name="marker.jpg"):
    # Define the ArUco dictionary
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
    
    # Create an empty image to store the marker
    marker_image = aruco.drawMarker(aruco_dict, id, size)
    
    # Save the marker image as a JPEG
    cv2.imwrite(file_name, marker_image)
    print(f"ArUco marker with ID {id} saved as {file_name}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate an ArUco marker.")
    parser.add_argument("--id", type=int, required=True, help="ID of the ArUco marker.")

    # Parse arguments
    args = parser.parse_args()
    
    # Example usage
    generate_aruco_marker(id=args.id, size=300)  # Generates a marker with ID 1, size 300x300
