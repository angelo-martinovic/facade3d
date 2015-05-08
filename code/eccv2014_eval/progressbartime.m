function progressbartime(n,N, txt, w)
% progressbar - display a progress bar with time estimate
%
%    progressbartime(n,N,w);
%
% displays the progress of n out of N.
% n should start at 1.
% cpu is the starting time of the process (cputime)
% w is the width of the bar (default w=50).
%
% AUTHOR: Gabriel Peyr, 2006, Hayko Riemenschneider 2014

if nargin<4
    w = 50;
end

if nargin<3
    txt = '';
end
% progress char
cprog = '.';
cprog1 = '*';
% begining char
cbeg = '[';
% ending char
cend = ']';

eta_unit = 's';
avg_unit = 's';
tot_unit = 's';


p = min( floor(n/N*(w+1)), w);

global pprev;
global cpu;
global ps;
if n==1
    pprev = -1;
    cpu = cputime;
    display('     ')
    %display('     ')
   % ps ='';
end

if not(p==pprev)
    tot = cputime-cpu;
    avg = tot / max(1,n-1);
    eta = avg * (N-n);
    tot = tot + eta;
    if n>1
        % clear previous string
        fprintf( repmat('\b', [1 length(ps)]) );
    end
    ps = repmat(cprog, [1 w]);
    ps(1:p) = cprog1;
    if(eta>3*60)
        eta = eta / 60;
        eta_unit = 'm';
    end
    if(avg>3*60)
        avg = avg / 60;
        avg_unit = 'm';
    end
    if(tot>3*60)
        tot = tot / 60;
        tot_unit = 'm';
    end
    
    if(eta>3*60)
        eta = eta / 60;
        eta_unit = 'h';
    end
    if(avg>3*60)
        avg = avg / 60;
        avg_unit = 'h';
    end
    if(tot>3*60)
        tot = tot / 60;
        tot_unit = 'h';
    end
    
    ps = [txt ' ' cbeg ps cend ' eta: ' num2str(eta,'%0.2f') eta_unit ' with avgtime: ' num2str(avg,'%0.2f') avg_unit ' tottime: ' num2str(tot,'%0.2f') tot_unit];
    fprintf(ps);
end
pprev = p;
if n==N
    fprintf('\n');
    clear global pprev ps;
end

