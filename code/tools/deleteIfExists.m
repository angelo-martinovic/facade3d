function deleteIfExists(filename)

    if exist(filename,'file')
        delete(filename);
    end
end