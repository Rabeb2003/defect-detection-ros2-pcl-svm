#!/bin/bash

echo "=========================================="
echo "Démonstration Visuelle - Preuves du Projet"
echo "=========================================="
echo ""

echo "1. Génération des graphiques de performance..."
python3 generate_performance_charts.py
echo ""

echo "2. Visualisation 3D des défauts..."
echo "   (Capture d'écran automatique)"
source /opt/ros/humble/setup.bash
source install/setup.bash
timeout 5 ros2 run defect_detection visualize_and_save
echo ""

echo "3. Liste des visualisations générées :"
echo ""
ls -lh visualizations/
echo ""

echo "=========================================="
echo "✓ Démonstration terminée !"
echo "=========================================="
echo ""
echo "Visualisations disponibles :"
echo "  📊 defect_visualization_3d.png - Vue 3D des 5 types de défauts"
echo "  📈 accuracy_per_class.png - Précision par classe"
echo "  📉 confusion_matrix.png - Matrice de confusion"
echo "  📊 dataset_distribution.png - Distribution du dataset"
echo "  ⚡ performance_metrics.png - Métriques de performance"
echo "  🎯 feature_importance.png - Importance des features"
echo "  ⏱️  processing_pipeline.png - Pipeline de traitement"
echo ""
echo "Pour voir le rapport complet :"
echo "  cat VISUAL_REPORT.md"
echo ""
echo "Pour visualiser en 3D interactif :"
echo "  source install/setup.bash"
echo "  ros2 run defect_detection visualize_defects"
