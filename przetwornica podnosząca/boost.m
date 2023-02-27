clear all;
close all;
clc;

V_in_min = 4;   %V
V_in_max = 14;  %V
V_o = 15;       %V
sprawnosc = 0.9;%-
I_o_max = 0.5;  %A
f = 100;        %KHz
T_A_max = 40;   %stopnie C
delta_V_in = 50;%mV
delta_V_o = 100;%mV

D_on_max = 1-sprawnosc*V_in_min/V_o %- 
I_in_max = I_o_max/(1-D_on_max)     %A
delta_i_L_max = 0.2*I_in_max        %A
L = V_o*1000/(4*delta_i_L_max*f)    %uH zaokrąglamy w dół

%katalog cewek

L = 82;         %uH podać mniejszą lub równą z katalogu
DCR = 0.071;    %podać z katalogu
delta_T_L = 20;  %stopnie C z katalogu
I_RMS = 2.75;    %A z katalogu

I_L_max = I_in_max + V_in_min*D_on_max/(2*L*f/1000)         %A
I_L_RMS = sqrt((I_in_max^2)+(delta_i_L_max/(2*sqrt(3)))^2)  %A
P_L_dc = I_in_max^2 * DCR                                   %W
P_L_ac = (delta_i_L_max/(2*sqrt(3)))^2 * 15 * DCR           %W
R_th_L = delta_T_L/(I_RMS^2 * DCR * (1+0.0039*delta_T_L))   %C/W
delta_T_L_max = (P_L_ac + P_L_dc) * R_th_L                  %stopnie C
T_L_max = T_A_max + delta_T_L_max                           %stopnie C
P_L = (P_L_dc + P_L_ac) * (1+0.0039*(T_L_max-25))           %W
T_L_max = T_A_max + P_L * R_th_L                            %stopnie C

%katalog diody
V_F = 0.39;     %napięcie dla I_L_max
V_F_th = 0.19;  %napięcie startu char
R_th_j_A_D = 140; %rezystancja termiczna diody

R_SD = (V_F - V_F_th)/I_L_max               %Ohm
I_D_RMS = I_o_max/sqrt(1-D_on_max)          %A
P_D_p = V_F_th * I_o_max + I_D_RMS^2 *R_SD  %W
T_j_D = T_A_max + R_th_j_A_D * P_D_p          %W

%katalog tranzystora
r_DS_on_25_C = 0.078;   %Ohm
delta_Q_g = 1.7 - 0.7;  %nC
R_gen = 12;             %Ohm
V_gen = 5;              %V
V_gs = 3.6;             %V
R_th_j_A_T = 100;       %C/W
Q_g = 2.5;              %nC

r_DS_on_70_C = 1.25 * r_DS_on_25_C                          %Ohm
I_k_RMS = I_o_max * sqrt(D_on_max) / (1  - D_on_max)        %A
P_k_P = I_k_RMS^2 * r_DS_on_70_C                            %W
t_fv = delta_Q_g * R_gen / (V_gen - V_gs)                   %ns
t_rv = delta_Q_g * R_gen / V_gs                             %ns
I_L_min = I_in_max - delta_i_L_max /2                       %A
P_k_K = ((t_fv*I_L_min*V_o/2)+(t_rv*I_L_max*V_o/2))*f/10^6  %W
P_k = P_k_P + P_k_K                                         %W
T_J_T = R_th_j_A_T *P_k + T_A_max                           %stopnie C
P_G = f * Q_g * V_gen/10^3                                  %mW

%kondensator z katalogu
C_in = 1000;   %uF
C_o = 1500

delta_i_L_max_2 = V_o/(4*L*f)                   %A
ESR_in = delta_V_in/(delta_i_L_max_2*1000)      %Ohm
I_C_i_RMS = delta_i_L_max_2/(2*sqrt(3))         %A
P_C_i = ESR_in * I_C_i_RMS^2                    %W
ESR_o = delta_V_o/(I_L_max*1000)                %Ohm
I_C_o_RMS = I_o_max*sqrt(D_on_max/(1-D_on_max)) %A
P_C_o = ESR_o * I_C_o_RMS^2                     %W

P_o = V_o * I_o_max                             %W
sprawnosc_2 = P_o/(P_o+P_L+P_k+P_D_p+P_C_o)     %-







