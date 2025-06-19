import cv2
import time
import os
import subprocess


def apply_zoom(frame, zoom_factor=1.5):
    h, w = frame.shape[:2]
    # Calcul des dimensions du crop
    new_w = int(w / zoom_factor)
    new_h = int(h / zoom_factor)
    x1 = (w - new_w) // 2
    y1 = (h - new_h) // 2
    # Découpe (crop)
    cropped = frame[y1:y1+new_h, x1:x1+new_w]
    # Redimensionne pour revenir à la taille d'origine
    zoomed = cv2.resize(cropped, (w, h))
    return zoomed

def main():
# Taille réelle de la scène (en cm)
    real_width_cm = 37.5
    real_height_cm = 28.3

    # Répertoires
    webcam_dir = "/home/mattis/ros_workshop_ws/src/vision_Yolo/vision_Yolo/webcam"
    yolo_dir = "/home/mattis/ros_workshop_ws/src/vision_Yolo/vision_Yolo/yolo"
    labels_dir = os.path.join(yolo_dir, "labels")

    os.makedirs(webcam_dir, exist_ok=True)
    os.makedirs(yolo_dir, exist_ok=True)
    os.makedirs(labels_dir, exist_ok=True)

    # Fichiers
    capture_image_path = os.path.join(webcam_dir, "capturedefault.jpg")
    yolo_image_name = "captureyolo.jpg"
    yolo_image_path = os.path.join(yolo_dir, yolo_image_name)
    yolo_txt_path = os.path.join(labels_dir, os.path.splitext(yolo_image_name)[0] + ".txt")

    # Capture de l'image
    cap = cv2.VideoCapture(2)

    if not cap.isOpened():
        print("Erreur : impossible d'ouvrir la caméra.")
    else:
        print("Caméra activée. Patientez 2 secondes...")
        time.sleep(2)

        ret, frame = cap.read()
        if ret:
            # Applique un zoom logiciel
            frame = apply_zoom(frame, zoom_factor=1.22)  # Modifiez le facteur selon vos besoins
            cv2.imwrite(capture_image_path, frame)
            print(f"Photo sauvegardée : {capture_image_path}")


            # YOLO
            yolo_command = [
                "yolo", "detect", "predict",
                "model=/home/mattis/ros_workshop_ws/src/vision_Yolo/vision_Yolo/train/weights/best.pt",
                f"source={capture_image_path}",
                f"project={yolo_dir}",
                f"name=.",
                "exist_ok=True",
                "save_txt=True"
            ]

            print("Lancement de YOLO...")
            try:
                result = subprocess.run(yolo_command, check=True, text=True, capture_output=True)
                print("YOLO terminé.")
                print(result.stdout)

                # Renommer les fichiers générés par YOLO
                # YOLO va générer un fichier avec le même nom que source -> capturedefault.png
                yolo_pred_img = os.path.join(yolo_dir, "capturedefault.jpg")
                yolo_pred_txt = os.path.join(yolo_dir, "labels", "capturedefault.txt")

                if os.path.exists(yolo_pred_img):
                    os.rename(yolo_pred_img, yolo_image_path)
                    print(f"Image YOLO renommée en : {yolo_image_path}")
                else:
                    print("❌ Image YOLO non trouvée :", yolo_pred_img)

                if os.path.exists(yolo_pred_txt):
                    os.rename(yolo_pred_txt, yolo_txt_path)
                    print(f"Fichier YOLO txt renommé en : {yolo_txt_path}")
                else:
                    print("❌ Fichier YOLO txt non trouvé :", yolo_pred_txt)

                # Afficher l'image annotée
                img_annotated = cv2.imread(yolo_image_path)
                if img_annotated is not None:
                    cv2.imshow("Image annotee YOLO", img_annotated)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()

                    # Calculer et afficher les positions des centres
                    img_h, img_w, _ = img_annotated.shape
                    if os.path.exists(yolo_txt_path):
                        with open(yolo_txt_path, "r") as f:
                            lines = f.readlines()

                        for line in lines:
                            parts = line.strip().split()
                            class_id = int(parts[0])
                            x_center_norm = float(parts[1])
                            y_center_norm = float(parts[2])

                            x_center_px = x_center_norm * img_w
                            y_center_px = y_center_norm * img_h

                            x_center_cm = x_center_px * (real_width_cm / img_w)
                            y_center_cm = y_center_px * (real_height_cm / img_h)

                            print(f"Classe {class_id} : centre = ({x_center_cm:.2f} cm, {y_center_cm:.2f} cm)")
                    else:
                        print("❌ Label file not found for center calculation.")
                else:
                    print("❌ Impossible de lire l'image annotée.")

            except subprocess.CalledProcessError as e:
                print("Erreur YOLO :")
                print(e.stderr)

        else:
            print("Erreur : impossible de capturer l'image.")

        cap.release()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()