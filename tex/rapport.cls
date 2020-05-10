\NeedsTeXFormat{LaTeX2e}

\ProvidesClass{rapport}

% classe de base

\LoadClass[a4paper, final, twoside]{article}

% extensions


\usepackage[utf8]{inputenc}
\usepackage[T1]{fontenc}
\usepackage[french]{babel}

\usepackage[nottoc]{tocbibind}

\usepackage[left=2cm,right=2cm,top=2cm,bottom=2cm]{geometry}
\usepackage{fancyhdr}
\usepackage{graphicx}
\usepackage{pdfpages}
\usepackage{array}
\usepackage{amsmath}
\usepackage{amssymb}

\usepackage{lipsum}

\usepackage[straightvoltages]{circuitikz}
\usetikzlibrary{babel}

\usepackage{tikz}\usetikzlibrary{automata, positioning, arrows}

\tikzset{->, 
    >=stealth', 
    node distance=5cm, 
    every state/.style={thin, fill=gray!5, font=\tiny}, 
    every node/.style={font=\small}, 
    initial text=$ $,
}

\usepackage[unicode=true, pagebackref=true, colorlinks=true, plainpages=true, linkcolor=black]{hyperref}

\pagestyle{fancy}

%random options

\parindent 0cm

%fancyhdr options
\renewcommand{\headrulewidth}{0.2pt}
\renewcommand{\footrulewidth}{0pt}
\fancyhead[R]{\includegraphics[width=2cm]{figures/epfl.png}}
\fancyhead[L]{Sound Locator}

\fancyfoot[C]{\thepage}



% commandes personnelles

\newcommand{\maketitre}{
\thispagestyle{empty}
\pagenumbering{gobble}
\vfill
\begin{center}
\includegraphics[width=5cm]{figures/epfl.png}\par%
\vspace{1 cm}
{\usefont{T1}{cmr}{bx}{n}
\fontsize{29.86pt}{29.86pt}\selectfont\@titre\\\vspace{0.5em}
\fontsize{16pt}{16pt}\selectfont\@soustitre\\\vspace{0.5em}
\large{\@groupe}\par}%
\vfill
\includegraphics[width=15cm]{figures/cover.jpg}\par%
\small{\color{gray!50}Montage photo du robot qui effectue sa tache (Réalisé a partir d'une vidéo).}\\
\vfill
\large
\@auteurA\\
\vspace{0.5cm}
\@auteurB\\
\vfill
{\color{gray!90 }Groupe 16}
\vfill
%Supervisé par \@assistant\ et \@prof\\ 
\vfill
{\large Rendu le 11 mai 2020}%
\vspace{1cm}
\newpage

\end{center}
\pagenumbering{arabic}
}


\newcommand{\pagevide}{
\null
\vfill
\newpage
}

\newcommand{\copyrightpage}{
\null
\vfill
\begin{center}
© Copyright \@auteurA, \@auteurB, \@auteurC, \@auteurD, \@auteurE\\
\end{center}
\newpage
}

%figure
%\fig{nom_fichier.extension}{label}{Légende de la figure}{Largeur (par ex: 0.6\linewidth)}{titre}
\newcommand{\fig}[5]{
   \begin{figure}[!ht]
   \begin{center}
    \textbf{#5}\par\medskip
   \includegraphics[width=#4]{figures/#1}
   \end{center}
   \caption{\label{#2}#3}
   \end{figure}   
} 

\newcommand{\figDeux}[6]{
   \begin{figure}[!ht]
   \begin{center}
    \textbf{#6}\par\medskip
   \includegraphics[width=#5]{figures/#1}
   \includegraphics[width=#5]{figures/#2}
   \end{center}
   \caption{\label{#3}#4}
   \end{figure}   
}

%equation
%\eqn{expression}{label}
\newcommand{\eqn}[2]{
	\begin{equation}
	#1
	\label{#2}
	\end{equation}
}


%math commands
\providecommand{\abs}[1]{\left\lvert#1\right\rvert}
\providecommand{\norm}[1]{\left\lVert#1\right\rVert}
\providecommand{\scal}[1]{\langle#1\rangle}
\providecommand{\todo}[1]{\large\colorbox{red}{#1}}




%variables personelles
\newcommand{\titre}{\def\@titre}
\newcommand{\soustitre}{\def\@soustitre}
\newcommand{\groupe}{\def\@groupe}
\newcommand{\assistant}{\def\@assistant}
\newcommand{\prof}{\def\@prof}
\newcommand{\auteur}[4]{\def\@auteurA{#1}
\def\@auteurB{#2}\def\@auteurC{#3}
\def\@auteurD{#4}}

