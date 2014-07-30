


%Copyright (c) 2014 Luke Marcus Biagio Testa
%All rights reserved.

%Redistribution and use in source and binary forms are permitted
%provided that the above copyright notice and this paragraph are
%duplicated in all such forms and that any documentation,
%advertising materials, and other materials related to such
%distribution and use acknowledge that the software was developed
%by the Luke Marcus Biagio Testa. The name of the
%Luke Marcus Biagio Testa may not be used to endorse or promote products derived
%from this software without specific prior written permission.
%THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
%IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
%WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.


clear all
close all
clc

data_path = ['C:\Users\thatguylukuss\Documents\Visual Studio 2012' ... 
                '\Projects\Kmeans_Serial - Copy\'];
cd(data_path)
         
k = [2 5 10 50 100 500 1000 5000 10000 50000];
time = [];

for i=1:size(k,2)

    val = 0;
    disp(['Running Clusters: ' num2str(k(i)) ])
    
        for j=1:3
            disp(['Run ' num2str(j)])
            
            [status, cout] = system([['Debug\Kmeans_Serial.exe '] ['Kmeans_Serial\Dataset.txt '] ... 
                       'C:\Users\thatguylukuss\Documents ' num2str(k(i)) ]);

                %% Get run time from command line

                temp = find( cout == ':' );
                start = temp(1) + 1;
                cout = cout(start:size(cout,2));
                temp = find( cout == 's' );
                cout = cout(1:temp(1) - 1);

                val = val + str2double(cout);
        end

    val = val / j;
    time = [time, val]

end

plot(time)
title('Execution Time on i7 4770 for Serail Kmeans, K = N and 500k points')
set(gca,'XTickLabel',k(1:size(time,2)));
xlabel('Clusters')
ylabel('Execution Time/ seconds')

cd('C:\Users\thatguylukuss\Documents\MATLAB\Parallel Architecture')