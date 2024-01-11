function [obj] = imm_output(obj,vx_ego) %#codegen
% IMM Kombination: Den finalen Zustand sch�tzen
%  
% Input:
% IMM: Datenstruktur
%   IMM(i).x: Zustand Modell i
%   IMM(i).P: Kovarianz Modell i
%   IMM(i).C: Wahrscheinlichkeit, dass nach der Interaktion das Objekt in Modell j ist
%
% Output:
% Xmerge: Zusammengef�hrter Zustand
% Pmerge: Zusammengef�hrte Kovarianz
%

anz_IMM_models = 3;

% Shortcut auf IMM-Struktur
IMM = obj.IMM;

% Varianz Orientierung und Drerate rausschreiben
obj.P_psi(1,1) = IMM(1).P(3,3);
obj.P_psi(2,2) = IMM(1).P(4,4);

% Transformation Modell 1 vom Constant-Turn-Rate zum karthesischen Zustand
[IMM(1).x, IMM(1).P] = turn2cartesian(IMM(1).x, IMM(1).P, vx_ego);

% Zustandsvektoren zusammenf�hren
x = zeros(size(IMM(1).x));
for j = 1:anz_IMM_models
    x = x + IMM(j).mu * IMM(j).x;   
end

% Kovarianzen zusammenf�hren
P = zeros(size(IMM(1).P));
for j = 1:anz_IMM_models
    P = P + IMM(j).mu * (IMM(j).P + (x - IMM(j).x)*(x - IMM(j).x)');   
end


% Output setzen
obj.P = P;
obj.x = x;

