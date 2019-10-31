function WriteReportCont(Reporter_Struct)
    filename = Reporter_Struct.contact_report.File;

    if (exist (filename) == 0)
        
        Fid = fopen(filename,'wt+');
        
        fw_string = sprintf('ModelForces\n');
        fprintf (Fid,fw_string);
        fw_string = sprintf('version=1\n');
        fprintf (Fid,fw_string);

        fw_string = sprintf('nRows=%d\n',length(Reporter_Struct.contact_report.time));
        fprintf (Fid,fw_string);
        
        fw_string = sprintf('nColumns=%d\n',length(Reporter_Struct.contact_report.dataLabls)+1);
        fprintf (Fid,fw_string);
        
        fw_string = sprintf('inDegrees=no\n');
        fprintf (Fid,fw_string);
        fw_string = sprintf('endheader\n');
        fprintf (Fid,fw_string);
        
        fw_string = sprintf('time');
        fprintf (Fid,fw_string);
        for i = 1:length(Reporter_Struct.contact_report.dataLabls)
            fw_string = sprintf('\t%s',Reporter_Struct.contact_report.dataLabls{i});
            fprintf (Fid,fw_string);
        end
        fw_string = sprintf('\n');
        fprintf (Fid,fw_string);

        for j = 1:length(Reporter_Struct.contact_report.time)
            
            fw_string = sprintf('%f',Reporter_Struct.contact_report.time(j));
            fprintf (Fid,fw_string);
            
            for i = 1:length(Reporter_Struct.contact_report.dataLabls) 
                fw_string = sprintf('\t%f',Reporter_Struct.contact_report.data(i,j));
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
        
        Prev_Data.Time = zeros(1,Prev_Data.nRows+length(Reporter_Struct.contact_report.time));
        Prev_Data.Data = zeros(Prev_Data.nColumns,Prev_Data.nRows+length(Reporter_Struct.contact_report.time));
        
        for j = 1:Prev_Data.nRows
            fr_string = fgets(Fid);
            CC = textscan(fr_string,'%f');
            
            Prev_Data.Time(1,j) = CC{1}(1);
            for i = 1:Prev_Data.nColumns
                Prev_Data.Data(i,j) = CC{1}(i+1);
            end
        end
        fclose(Fid);
        
        if Prev_Data.Time(Prev_Data.nRows) == Reporter_Struct.contact_report.time(1)
            for j = 2:length(Reporter_Struct.contact_report.time)
                Prev_Data.Time(1,j+Prev_Data.nRows-1) = Reporter_Struct.contact_report.time(j);
            end

            for i = 1:Prev_Data.nColumns
                address = 1;
                for j = 1:length(Reporter_Struct.contact_report.dataLabls)
                    if strcmp(Prev_Data.Labels{i},Reporter_Struct.contact_report.dataLabls{j})
                        address = j;
                        break;
                    end
                end

                for j = 2:length(Reporter_Struct.contact_report.time)
                    Prev_Data.Data(i,j+Prev_Data.nRows-1) = Reporter_Struct.contact_report.data(address,j);
                end
            end
            Prev_Data.Time(:,end) = [];
            Prev_Data.Data(:,end) = [];
        else
            for j = 1:length(Reporter_Struct.contact_report.time)
                Prev_Data.Time(1,j+Prev_Data.nRows) = Reporter_Struct.contact_report.time(j);
            end

            for i = 1:Prev_Data.nColumns
                address = 1;
                for j = 1:length(Reporter_Struct.contact_report.dataLabls)
                    if strcmp(Prev_Data.Labels{i},Reporter_Struct.contact_report.dataLabls{j})
                        address = j;
                        break;
                    end
                end

                for j = 1:length(Reporter_Struct.contact_report.time)
                    Prev_Data.Data(i,j+Prev_Data.nRows) = Reporter_Struct.contact_report.data(address,j);
                end
            end
        end
            
        delete (filename);

        Fid = fopen(filename,'wt+');
        
        fw_string = sprintf('ModelForces\n');
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