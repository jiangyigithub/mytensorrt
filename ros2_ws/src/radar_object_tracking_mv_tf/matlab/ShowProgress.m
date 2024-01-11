% function ShowProgress(Maximum, CurrentValue, [Step = 5])
%
% Fortschrittsanzeige
% \param Step not used
function ShowProgress(Maximum, CurrentValue, Step, sTitle)

	global g_dProgressLastCPUTime
	global g_nProgressK1
	global g_hProgressWindows
	
%% Demo

	if nargin == 0
		clc
		N = 100000;
		for n = 1:N
			ShowProgress(N, n, [], 'Example: for n=1:50000');
		end
		
		return
	end
	
%% Los gehts

	if nargin < 4, sTitle = 'Please wait...'; end
	
	if Maximum <= 1
		return
	end
	
	% Neustart
	if isempty(g_nProgressK1) ...
			|| isempty(g_dProgressLastCPUTime)...
			|| CurrentValue < g_nProgressK1 ...
			|| isempty(g_hProgressWindows)...
			||~ishandle(g_hProgressWindows)
		
		g_nProgressK1 = 0;
		g_dProgressLastCPUTime = cputime();
		
		
% 		g_hProgressWindows = waitbar(0, sTitle, ...
% 			'CreateCancelBtn', @Chancel);

 		if		isempty(g_hProgressWindows)...
			||	~ishandle(g_hProgressWindows)
			g_hProgressWindows = waitbar(0, sTitle);
		end
		
		set(g_hProgressWindows, 'Name', sTitle);
	end
	
	if CurrentValue >= Maximum
		% Fenster löschen temporär deaktiviert, denn sonst kommt immer ein neues in den Vordergrund
		% und erschwert das weitere Arbeiten in Windows
		delete(g_hProgressWindows)
		return
	end
	
		
	% Vergangene Zeit
	dCPUTime = cputime();
	dDiffTime = dCPUTime - g_dProgressLastCPUTime;

	if dDiffTime >= 1 && (CurrentValue > g_nProgressK1)
		% Fortschritt
		nDiffValues = CurrentValue - g_nProgressK1;

		% Lineare Extrapolation, wie lange es noch dauert
		dTimeRemaining = (Maximum - CurrentValue) * (dDiffTime / nDiffValues);

		s = sprintf('%s\n%d sec left, ETA: %s', ...
				sTitle, ...
				round(dTimeRemaining), ...
				datestr(now + dTimeRemaining / 3600 / 24, 'HH:MM:SS'));

		waitbar(CurrentValue/Maximum, g_hProgressWindows, s, ...
			'Interpreter', 'None');

		g_dProgressLastCPUTime = dCPUTime;

		% Letzten Wert merken, für den die Info kam
		g_nProgressK1 = CurrentValue;
	end
end


function Chancel(object, src, event)
	global g_hProgressWindows 
	delete(g_hProgressWindows);
% 	disp('Abgebrochen');
% 	dbquit('all')
	assert(0)
% 	Abbruch
end



