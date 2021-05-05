##########################################################
#                                                        #
# EXTRACTION DES DONNEES DE MESURE DE RAYON DE COURBURE  #
#                                                        #
# Ce fichier a pour but d'extraire les données mesurées  #
# par la centrale inertielle et receuilli sur la         #
# Raspberry dans un fichier texte resultat.txt.          #
#                                                        #
##########################################################

#----import des librairies nécessaires--------------------
import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
from pathlib import Path
from scipy.optimize import curve_fit
pathlist = Path("./").glob('**/*.txt')
myfile = 'filename.txt'

#----Ouverture de résultat et inscription du nom des colonnes


g = open("resultat.txt", 'a')
g.writelines("Vitesse_commande, Angle_commande, acc_mean, psi_mean, Vitesse_reelle, Courbure \n")
g.close()

#----Parcours de tous les fichiers du répertoire actuel 
#----comprenant la chaine de caractère "angle=" dans leur
#----titre. Cela permet de ne sélectionner que les fichier
#----de mesure.

for path in pathlist :	
	nom = path.name # nom du fichier étudié
	if "angle=" in nom : 
		# Détermination des position de l'angle commande 
		# et de la vitesse commande dans le titre du fichier.
		# nous avions choisis un titre de format
		# "donnees_cercle_Fri Apr  2 15_14_46 2021_angle=-5_vitesse=17"
		c1 = nom.find("angle=") + 6
		c2 = nom.find("_vitesse=")
		c3 = c2 + 9
		c4 = nom.find(".txt")
		# Sélection de l'angle et de la vitesse commande
		angle = nom[c1:c2]
		vitesse = nom[c3:c4]

		#----Ouverture et lecture du fichier
		f = open (nom, 'r')
		data = f.readlines()
		f.close()
		
		#----Extraction des données
		data = np.array(data)
		data2 =[]
		temps=[]
		acceleration=[]
		vitesse_lacet=[]
		for ligne in data:
			np.array(ligne)
			ligne2 = ligne.split(",")
			temps.append(float(ligne2[0]))	
			acceleration.append(float(ligne2[1]))
			vitesse_lacet.append(float(ligne2[6]))
		plt.plot(temps, vitesse_lacet)

		#----Calcul de données moyennes
		tmin, tmax  = temps[0], temps[-1]	
		time = np.arange(tmin, tmax, 0.01)
		f = interpolate.interp1d(temps, acceleration, axis=0)
		acc= f(time)
		f=interpolate.interp1d(temps, vitesse_lacet, axis=0)
		psi = f(time)
		acc_mean = acc.mean() * 9.81
		psi_mean = psi.mean() * np.pi / 180
		vit_reelle = acc_mean/psi_mean
		courbure = acc_mean * psi_mean**2

		#----Inscription des résultats dans le fichier "résultat.txt"
		g = open("resultat.txt", 'a')
		g.writelines(vitesse + " , " + angle + " , " + str(acc_mean) + " , " + str(psi_mean) + " , " + str(vit_reelle) + " , " + str(courbure) + " \n")
		g.close()
