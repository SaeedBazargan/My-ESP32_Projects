import gradio as gr
from gradio_image_annotation import image_annotator
# import tensorflow as tf
import os
import io
from PIL import Image
import json
import csv

# Global state
images_list = []     # List of image file paths
annotations = []     # Parallel list of annotations (list of box dicts) for each image
current_index = 0    # Index of current image being annotated

def reset_state():
    """Reset the global state for a new set of images."""
    global images_list, annotations, current_index
    images_list = []
    annotations = []
    current_index = 0

def load_images(files):
    """
    Callback for loading images and initializing annotation state.
    'files' is a list of uploaded file paths.
    Returns initial annotation data and status text.
    """
    global images_list, annotations, current_index

    # Reset state
    reset_state()

    # Input validation: check files
    if not files:
        raise gr.Error("No files uploaded. Please upload at least one image.")

    # Validate and filter images by extension
    supported_ext = (".jpg", ".jpeg", ".png", ".bmp", ".gif")
    valid_images = []
    for file in files:
        ext = os.path.splitext(file)[1].lower()
        if ext not in supported_ext:
            # Skip unsupported file but continue
            continue
        valid_images.append(file)
    if not valid_images:
        raise gr.Error("No valid image files found. Supported formats: JPG, PNG, BMP, GIF.")

    # Initialize global image list and empty annotations
    images_list = valid_images
    annotations = [[] for _ in images_list]
    current_index = 0

    # Prepare initial annotation dict for the first image
    first_image_path = images_list[0]
    first_annotation = {"image": first_image_path, "boxes": []}

    status_text = f"Image 1 of {len(images_list)}"
    return first_annotation, status_text

def save_current_annotations(annotation_data):
    """
    Helper to save the current annotation from the image_annotator component
    into the global annotations list at current_index.
    'annotation_data' is the dict returned by the annotator (contains 'boxes').
    """
    global annotations, current_index
    if annotation_data and "boxes" in annotation_data:
        boxes = []
        for box in annotation_data["boxes"]:
            # Copy relevant fields
            box_copy = {
                "xmin": box["xmin"],
                "ymin": box["ymin"],
                "xmax": box["xmax"],
                "ymax": box["ymax"]
            }
            boxes.append(box_copy)
        annotations[current_index] = boxes

def navigate_image(annotation_data, direction):
    """
    Callback for navigating between images.
    'direction' should be +1 for next or -1 for previous.
    Saves current annotations and moves index.
    Returns updated annotation data for new image and status text.
    """
    global current_index, images_list

    # Save current annotations
    save_current_annotations(annotation_data)

    # Update index
    new_index = current_index + direction
    # Bounds check
    if new_index < 0 or new_index >= len(images_list):
        # Out of range: do nothing
        new_index = current_index
    current_index = new_index

    # Prepare annotation dict for new image
    image_path = images_list[current_index]
    image_boxes = annotations[current_index] if annotations[current_index] else []
    annot_dict = {"image": image_path, "boxes": image_boxes}

    status = f"Image {current_index+1} of {len(images_list)}"
    return annot_dict, status

def export_annotations(format_choice):
    """
    Export the collected annotations to the chosen format and return filepath.
    Supported formats: 'CSV', 'JSON', 'TFRecord'.
    """
    global images_list, annotations

    if not images_list:
        raise gr.Error("No images to export. Please annotate images first.")
    
    fmt = format_choice.lower()
    if fmt not in ("csv", "json", "tfrecord"):
        raise gr.Error("Unsupported format. Choose CSV, JSON, or TFRecord.")

    output_path = f"annotations.{fmt}"
    
    if fmt == "csv":
        # Write CSV with columns: filename, xmin, ymin, xmax, ymax
        with open(output_path, mode="w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["filename", "xmin", "ymin", "xmax", "ymax"])
            for img_path, boxes in zip(images_list, annotations):
                filename = os.path.basename(img_path)
                for box in boxes:
                    writer.writerow([filename, box["xmin"], box["ymin"], box["xmax"], box["ymax"]])
    
    elif fmt == "json":
        # Write JSON as list of {image: ..., annotations: [...]}
        data = []
        for img_path, boxes in zip(images_list, annotations):
            item = {
                "image": os.path.basename(img_path),
                "annotations": []
            }
            for box in boxes:
                ann = {
                    "xmin": box["xmin"],
                    "ymin": box["ymin"],
                    "xmax": box["xmax"],
                    "ymax": box["ymax"]
                }
                item["annotations"].append(ann)
            data.append(item)
        with open(output_path, mode="w") as jsonfile:
            json.dump(data, jsonfile, indent=2)
        
    return output_path

# Build Gradio interface
with gr.Blocks() as demo:
    gr.Markdown("## Image Annotation Tool")
    gr.Markdown("Upload images (JPG/PNG/BMP/GIF), draw bounding boxes, and export annotations.")
    with gr.Row():
        upload_files = gr.File(label="Upload Images", file_count="multiple", type="filepath")
        load_button = gr.Button("Load Images")
    status_label = gr.Label(value="No image loaded.")
    with gr.Row():
        prev_button = gr.Button("Previous")
        next_button = gr.Button("Next")
    # Annotator component for drawing bounding boxes
    annotator = image_annotator(height=600)
    with gr.Row():
        export_format = gr.Radio(choices=["CSV", "JSON", "TFRecord"], label="Export Format", value="CSV")
        export_button = gr.Button("Export Annotations")
    download_output = gr.File(label="Download Annotations")
    # Connect events
    load_button.click(fn=load_images, inputs=[upload_files], outputs=[annotator, status_label])
    prev_button.click(fn=lambda ann: navigate_image(ann, -1), inputs=[annotator], outputs=[annotator, status_label])
    next_button.click(fn=lambda ann: navigate_image(ann, 1), inputs=[annotator], outputs=[annotator, status_label])
    export_button.click(fn=export_annotations, inputs=export_format, outputs=download_output)

if __name__ == "__main__":
    demo.launch()
