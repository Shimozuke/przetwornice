clear all;
close all;
clc;

%założenia podstawowe
V_o = 5;            %V
I_o_max = 3;        %A
V_in_min = 10;       %V
V_in_max = 30;      %V
delta_V_o = 50;     %mV
delta_V_in = 200;   %mV
f = 100;            %kHz
sprawnosc = 0.78;   %założona sprawność
J = 5;              %A/mm^2
T_a = 40;           %stopni C

delta_i_L1_max = 0.2 * I_o_max                              %A
D_min = V_o/(V_in_max*sprawnosc)                            %-
D_max = V_o/V_in_min/sprawnosc
L1 = V_o*(1-D_min)*1000/(delta_i_L1_max*f)                  %uH
i_L_max1 = I_o_max + delta_i_L1_max/2                       %A
I_o_RMS_max = sqrt(I_o_max^2+(delta_i_L1_max/(2*sqrt(3)))^2)%A
A_W = I_o_RMS_max/J                                         %mm^2
d_W = sqrt(4*A_W/pi)                                        %mm (zaokrąglij do 1/10 mm)

%zaokrąglenie do 1/10 mm
d_W = 0.9;

I2L = i_L_max1^2 * L1 /1000                             %mJ sprawdzam g z wykresu I^2*L

%z katalogu rdzenia
A_L = 315;          %uH/zwój^2
W_A = 30.9;         %mm^2
A_min = 55.4;       %mm^2
ue = 151;           %-
u0 = 4*pi*10^(-7);  %-
l_e = 38.4;         %mm
V_e = 2440;         %mm^3
W_W = 8.6;          %mm
K_i = 0.9;         %-
MLT = 42;           %mm
ro_miedz = 2.0*10^(-8)%Ohm*n

N = sqrt(L1*1000/A_L)                                   %liczba zwojów

%całkowita liczba zwojów(zaokrąglenie w dół)
N = 14;

k_C_u = N*A_W/W_A                                       %sprawdź czy mniejsze od 0.5

L = N^2*A_L/1000                                        %uH indukcyjność zaprojektowabegi dławika

B_c_max = L*i_L_max1/(N*A_min)                          %T sprawdź czy mniejsze niż 0.3
delta_i_L_max = V_o*(1-D_min)*1000/(L*f)                %A
I_L_max = I_o_max + delta_i_L_max/2                     %A
delta_i_L = V_o*(1-D_max)*1000/(L*f)                    %A
B_m = u0*ue*(N*delta_i_L*10^6/2/l_e)                    %mT
P_v = 65 * 10^(-6) * f^1.3 * (B_m/1000)^2.5             %mW/mm^3
P_r = P_v*V_e                                           %mW

N_W = W_W*K_i/d_W                                       %liczba zwojów na warstwę
%całkowita liczba
N_W = 8;

M = N/N_W                                               %liczba warstw
%całkowita liczba
M = 2;

M_max = 3.5*K_i/d_W                                     %liczba warstw maksylamna

l_w = MLT*N/1000                                        %m długość uzwojenia
R_uzw_dc = l_w*ro_miedz/(pi*1.1^2*10^(-6)/4)       %Ohm (za cholore nie moge rozgryźć tego wzoru)
P_uzw_dc = I_o_max^2 * R_uzw_dc*1000                        %mW
ro_w = 71.39/sqrt(f*1000)                               %mm
A1 = 0.834*(d_W*sqrt(K_i)/ro_w)                   %-
F_R1 = A1 * (2 * M^2 + 1)/3                             %-
R_uzw_ac1 = F_R1 * R_uzw_dc                             %Ohm
delta_i_L_RMS = delta_i_L/(2*sqrt(3))                   %A
P_uzw_ac = delta_i_L_RMS^2 * R_uzw_ac1*1000             %mW

P_uzw = P_uzw_ac + P_uzw_dc                             %mW

P_L = P_uzw + P_r                                       %mW

%z katalogu rdzenia
R_th_r = 57;                                            %K/W

delta_T_r = R_th_r * P_L/1000                           %stopnie C
T_r = delta_T_r + T_a                                   %stopnie C

I_K_RMS = I_o_max * sqrt(D_max)                         %A
I_K_avg = D_max * I_o_max                               %A

%z katalogu tranzystora
V_K_min = 1.5;  %V
V_K_max = 2.2;  %V
I_K_max = 6;    %A

r_K = (V_K_max-V_K_min)/I_K_max                         %Ohm
P_K_p = (V_K_min * I_K_avg + I_K_RMS^2 * r_K) * 1000    %mW

I_K_min = I_o_max-delta_i_L/2
I_K_max = I_o_max+delta_i_L/2

P_K_k = f*(V_in_min*I_K_min*(50+3*I_K_min)+V_in_min*I_K_max*(50+3*I_K_max))/1000    %mW

P_K = P_K_p + P_K_k                                     %mW

I_D_avg = (1-D_max)*I_o_max                             %A
I_D_RMS = I_o_max*sqrt(1-D_max)                         %A

%z katalogu diody
V_F_th = 1;   %V
V_F_i_max = 1; %V

R_SD = (V_F_i_max - V_F_th)/I_L_max                     %Ohm

P_D_p = (V_F_th * I_D_avg + I_D_RMS^2 * R_SD) * 1000    %mW

ESR_in = delta_V_in/I_L_max                             %mOhm
ESR_out = delta_V_o/delta_i_L_max                       %mOhm

sprawnosc2 = I_o_max*V_o*1000/(I_o_max*1000*V_o+P_K+P_L+P_D_p)    %-












