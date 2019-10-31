function WriteStates(State_Storag)
    filename = State_Storag.File33;

    if (exist (filename) == 0)
        
        Fid = fopen(filename,'wt+');
        
        fw_string = sprintf('ModelStates\n');
        fprintf (Fid,fw_string);
        fw_string = sprintf('version=1\n');
        fprintf (Fid,fw_string);

        fw_string = sprintf('nRows=%d\n',length(State_Storag.time));
        fprintf (Fid,fw_string);
        
        fw_string = sprintf('nColumns=%d\n',length(State_Storag.name_v33)+1);
        fprintf (Fid,fw_string);
        
        fw_string = sprintf('inDegrees=no\n');
        fprintf (Fid,fw_string);
        fw_string = sprintf('endheader\n');
        fprintf (Fid,fw_string);
        
        fw_string = sprintf('time');
        fprintf (Fid,fw_string);
        for i = 1:length(State_Storag.name_v33)
            fw_string = sprintf('\t%s',State_Storag.name_v33{i});
            fprintf (Fid,fw_string);
        end
        fw_string = sprintf('\n');
        fprintf (Fid,fw_string);

        for j = 1:length(State_Storag.time)
            
            fw_string = sprintf('%f',State_Storag.time(j));
            fprintf (Fid,fw_string);
            
            for i = 1:length(State_Storag.name_v33) 
                fw_string = sprintf('\t%f',State_Storag.data(i,j));
                fprintf (Fid,fw_string);
            end
            fw_string = sprintf('\n');
            fprintf (Fid,fw_string);
        end        
        
        fclose(Fid);
        
        
    else
        
        Prev_Data = struct();
        
        Fid = fopen(filename,'rt');
        fgets(Fid);
        fgets(Fid);
        
        fr_string = fgets(Fid);
        SplitStr = split(fr_string,'=');        
        Prev_Data.nRows = sscanf(SplitStr{2},'%d');
        
        fr_string = fgets(Fid);
        SplitStr = split(fr_string,'=');        
        Prev_Data.nColumns = sscanf(SplitStr{2},'%d')-1;
        
        fgets(Fid);
        fgets(Fid);
        
        Prev_Data.Labels = cell(Prev_Data.nColumns,1);
        
        fr_string = fgets(Fid);
        CC = textscan(fr_string,'%s\t');
        for i = 1:Prev_Data.nColumns
            Prev_Data.Labels{i} = CC{1}{i+1};
        end
        
        Prev_Data.Time = zeros(1,Prev_Data.nRows+length(State_Storag.time));
        Prev_Data.Data = zeros(Prev_Data.nColumns,Prev_Data.nRows+length(State_Storag.time));
        
        for j = 1:Prev_Data.nRows
            fr_string = fgets(Fid);
            CC = textscan(fr_string,'%f');
            
            Prev_Data.Time(1,j) = CC{1}(1);
            for i = 1:Prev_Data.nColumns
                Prev_Data.Data(i,j) = CC{1}(i+1);
            end
        end
        fclose(Fid);
        
        if Prev_Data.Time(Prev_Data.nRows) == State_Storag.time(1)
            for j = 2:length(State_Storag.time)
                Prev_Data.Time(1,j+Prev_Data.nRows-1) = State_Storag.time(j);
            end

            for i = 1:Prev_Data.nColumns
                address = 1;
                for j = 1:length(State_Storag.name_v33)
                    if strcmp(Prev_Data.Labels{i},State_Storag.name_v33{j})
                        address = j;
                        break;
                    end
                end

                for j = 2:length(State_Storag.time)
                    Prev_Data.Data(i,j+Prev_Data.nRows-1) = State_Storag.data(address,j);
                end
            end
            Prev_Data.Time(:,end) = [];
            Prev_Data.Data(:,end) = [];
        else
            for j = 1:length(State_Storag.time)
                Prev_Data.Time(1,j+Prev_Data.nRows) = State_Storag.time(j);
            end

            for i = 1:Prev_Data.nColumns
                address = 1;
                for j = 1:length(State_Storag.name_v33)
                    if strcmp(Prev_Data.Labels{i},State_Storag.name_v33{j})
                        address = j;
                        break;
                    end
                end

                for j = 1:length(State_Storag.time)
                    Prev_Data.Data(i,j+Prev_Data.nRows) = State_Storag.data(address,j);
                end
            end
        end
            
        delete (filename);

        Fid = fopen(filename,'wt+');
        
        fw_string = sprintf('ModelStates\n');
        fprintf (Fid,fw_string);
        fw_string = sprintf('version=1\n');
        fprintf (Fid,fw_string);

        fw_string = sprintf('nRows=%d\n',length(Prev_Data.Time));
        fprintf (Fid,fw_string);
        
        fw_string = sprintf('nColumns=%d\n',length(Prev_Data.Labels)+1);
        fprintf (Fid,fw_string);
        
        fw_string = sprintf('inDegrees=no\n');
        fprintf (Fid,fw_string);
        fw_string = sprintf('endheader\n');
        fprintf (Fid,fw_string);
        
        fw_string = sprintf('time');
        fprintf (Fid,fw_string);
        for i = 1:length(Prev_Data.Labels)
            fw_string = sprintf('\t%s',Prev_Data.Labels{i});
            fprintf (Fid,fw_string);
        end
        fw_string = sprintf('\n');
        fprintf (Fid,fw_string);

        for j = 1:length(Prev_Data.Time)
            
            fw_string = sprintf('%f',Prev_Data.Time(j));
            fprintf (Fid,fw_string);
            
            for i = 1:length(Prev_Data.Labels) 
                fw_string = sprintf('\t%f',Prev_Data.Data(i,j));
                fprintf (Fid,fw_string);
            end
            fw_string = sprintf('\n');
            fprintf (Fid,fw_string);
        end        
        
        fclose(Fid);
        
    end

%     Last_Time = State_Storag.time(end);
%     Last_Data = State_Storag.data(:,end)';
    
end