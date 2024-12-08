import cv2
import cv2.aruco as aruco
import argparse


# Function to generate and save an ArUco marker
def generate_aruco_marker(ids, size=300):
    # Define the ArUco dictionary
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)

    for i, id in enumerate(ids):
        file_name = f'marker-{i + 1}.jpg'
        # Create an empty image to store the marker
        marker_image = aruco.drawMarker(aruco_dict, id, size)

        # Save the marker image as a JPEG
        cv2.imwrite(file_name, marker_image)
        print(f"ArUco marker with ID {id} saved as {file_name}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate ArUco markers.")
    parser.add_argument("--ids", type=int, nargs='+', required=True, help="ID array of the ArUco markers.")

    # Parse arguments
    args = parser.parse_args()
    
    # Example usage
    generate_aruco_marker(ids=args.ids, size=300)  # Generates a marker with ID 1, size 300x300
