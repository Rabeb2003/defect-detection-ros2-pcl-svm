# Détection de Défauts en Temps Réel pour l'Industrie 4.0

Projet ROS2 Humble utilisant PCL (Point Cloud Library) pour la détection automatique de défauts sur pièces manufacturées.

## 🎯 Objectif

Système de vision 3D pour l'inspection automatisée en temps réel capable de :
- Scanner des pièces en production
- Comparer avec des modèles CAD de référence
- Détecter et classifier les défauts (rayures, déformations, erreurs dimensionnelles)
- Générer des rapports de qualité instantanés

## 🏗️ Architecture

```
[Capteur 3D] → [Acquisition PCL] → [Prétraitement] → [Registration CAD] 
→ [Segmentation] → [Extraction Features] → [Classification SVM/ML] 
→ [Rapport Qualité] → [Interface ROS2]
```

## 📦 Modules

### 1. PointCloudAcquisition
- Acquisition de nuages de points depuis capteurs ou fichiers
- Filtrage du bruit (Statistical Outlier Removal)
- Downsampling (Voxel Grid)

### 2. CADRegistration
- Chargement de modèles CAD (fichiers PCD)
- Registration ICP (Iterative Closest Point)
- Registration NDT (Normal Distributions Transform)

### 3. DefectSegmentation
- Segmentation de surfaces planes
- Clustering Euclidien
- Calcul de cartes de distance

### 4. DefectClassifier
- **Classification SVM avec OpenCV ML**
- Extraction de caractéristiques géométriques (8 features)
- Classification basée sur règles (fallback)
- Types de défauts : normal, dent, bulge, rayure, fissure
- **Précision d'entraînement : 78%**

## 🤖 Machine Learning Integration

### Dataset d'entraînement
- **50 échantillons générés** avec défauts simulés
- 5 classes : normal, dent, bulge, scratch, crack
- Format PCD avec prétraitement intégré

### Pipeline d'entraînement
```bash
# Générer données d'entraînement
ros2 run defect_detection generate_training_data

# Entraîner le modèle SVM
ros2 run defect_detection train_svm_classifier
```

### Features extraites (8 dimensions)
1. Nombre de points
2-4. Dimensions de la bounding box (x, y, z)
5. Volume
6. Coordonnée Z du centroïde
7. Courbure moyenne (normales)
8. Écart-type de Z

### Modèle SVM
- Kernel : RBF (Radial Basis Function)
- C = 1.0
- Gamma = 0.1
- Sauvegardé dans `data/svm_model.xml`

## 🚀 Installation

### Prérequis
- Ubuntu 22.04
- ROS2 Humble
- PCL 1.12
- C++17

### Compilation

```bash
cd ~/defect_detection_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 📖 Utilisation

### Test rapide du système

```bash
# Script de test complet
./test_system.sh
```

### Lancement manuel avec données simulées

```bash
# Terminal 1 - Publisher de nuages de points simulés
ros2 run defect_detection cloud_publisher

# Terminal 2 - Node de détection de défauts
ros2 run defect_detection defect_detector

# Terminal 3 - Visualisation (optionnel)
ros2 run rviz2 rviz2
```

### Lancement avec launch file

```bash
ros2 launch defect_detection defect_detection.launch.py
```

### Avec modèle CAD

```bash
ros2 launch defect_detection defect_detection.launch.py cad_model_path:=/path/to/model.pcd
```

### Entraînement personnalisé

```bash
# Générer nouvelles données d'entraînement
ros2 run defect_detection generate_training_data

# Entraîner le modèle SVM
ros2 run defect_detection train_svm_classifier
```

## 🔧 Configuration

Les paramètres sont configurables dans `config/defect_detection_params.yaml` :

```yaml
defect_detection_node:
  ros__parameters:
    cad_model_path: ""              # Chemin vers modèle CAD
    use_icp: true                   # Méthode de registration
    enable_classification: true      # Activer classification
    publish_markers: true           # Publier markers RViz
    
    # Paramètres acquisition
    outlier_mean_k: 50
    voxel_leaf_size: 0.01
    
    # Paramètres registration
    icp_max_iterations: 50
    icp_max_correspondence_distance: 0.05
    
    # Paramètres segmentation
    cluster_tolerance: 0.02
    min_cluster_size: 100
    
    # Paramètres classification
    scratch_curvature_threshold: 0.5
    dent_depth_threshold: 0.02
```

## 📡 Topics ROS2

### Subscribers
- `/input_cloud` (sensor_msgs/PointCloud2) - Nuage de points d'entrée

### Publishers
- `/processed_cloud` (sensor_msgs/PointCloud2) - Nuage traité
- `/defects_detected` (std_msgs/String) - Résultats de détection
- `/defect_markers` (visualization_msgs/MarkerArray) - Markers de visualisation

## 🧪 Tests

### Données de test simulées

Le node `cloud_publisher` génère des nuages de points synthétiques avec :
- Un plan de base
- Une bosse simulée (dent)
- Une rayure simulée

### Visualisation dans RViz

1. Lancer RViz : `rviz2`
2. Ajouter display : `PointCloud2`
3. Topic : `/processed_cloud`
4. Ajouter display : `MarkerArray`
5. Topic : `/defect_markers`

## 📊 Performance

- **Temps de traitement** : < 100ms par nuage de points
- **Précision SVM** : 78% (50 échantillons d'entraînement)
- **Types de défauts** : 5 classes (normal, dent, bulge, scratch, crack)
- **Détection en temps réel** : ~10 nuages/seconde
- **Features extraites** : 8 dimensions géométriques

## 🔬 Pour Mitacs

### Proposition de projet

**Titre** : Système de Vision 3D Intelligent pour l'Inspection Automatisée de Pièces Manufacturées en Temps Réel

**Objectifs** :
- Réduction des coûts de contrôle qualité de 40%
- Inspection en temps réel (< 1 seconde/pièce)
- Précision de détection > 95%

**Partenaires potentiels** :
- Bombardier (aéronautique)
- Stellantis (automobile)
- Celestica (électronique)

**Timeline** : 4 mois
- Mois 1 : Acquisition et prétraitement
- Mois 2 : Registration CAD
- Mois 3 : Segmentation et classification
- Mois 4 : Intégration et déploiement

## 📝 Structure du projet

```
defect_detection_ws/
├── src/
│   
└── defect_detection/
    ├── include/defect_detection/
    │   ├── point_cloud_acquisition.hpp
    │   ├── cad_registration.hpp
    │   ├── defect_segmentation.hpp
    │   └── defect_classifier.hpp
    ├── src/
    │   ├── modules/
    │   │   ├── point_cloud_acquisition.cpp
    │   │   ├── cad_registration.cpp
    │   │   ├── defect_segmentation.cpp
    │   │   └── defect_classifier.cpp
    │   ├── defect_detector.cpp
    │   ├── cloud_publisher.cpp
    │   ├── generate_training_data.cpp
    │   └── train_svm_classifier.cpp
    ├── config/
    │   └── defect_detection_params.yaml
    ├── launch/
    │   ├── defect_detection.launch.py
    │   └── test_simulation.launch.py
    ├── CMakeLists.txt
    └── package.xml
├── data/
│   ├── training/           # 50 échantillons PCD
│   ├── svm_model.xml      # Modèle SVM entraîné
│   └── PCL_Classification/ # Référence GitHub
└── test_system.sh         # Script de test
```

## 🛠️ Développement futur

- [ ] Intégration deep learning pour classification
- [ ] Support de capteurs réels (LiDAR, caméras depth)
- [ ] Interface web pour visualisation
- [ ] Base de données pour traçabilité
- [ ] Intégration MES/ERP

## 📄 Licence

BSD 3-Clause

## 👥 Auteurs

Développé pour projet Mitacs - Détection de défauts Industrie 4.0

## 🙏 Remerciements

- Point Cloud Library (PCL)
- ROS2 Humble
- Open Perception
