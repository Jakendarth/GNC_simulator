import tabula
# Pages of each table
table_info = {
    "Table 1": {"pages": [17, 18], "heel": 0, "trim": -1.5},
    "Table 2": {"pages": [19, 20], "heel": 0, "trim": -1.0},
    "Table 3": {"pages": [21, 22], "heel": 0, "trim": -0.5},
    "Table 4": {"pages": [23, 24], "heel": 0, "trim": 0},
    "Table 5": {"pages": [25, 26], "heel": 0, "trim": 0.5},
    "Table 6": {"pages": [27, 28], "heel": 0, "trim": 1.0},
    "Table 7": {"pages": [29, 30], "heel": 0, "trim": 1.5}
}
# Vessel data
tito_neri={"m":16.9,"xg":0,"Iz":0.51,"Xudot":-1.2,"Yvdot":-49.2,"Yrdot":0,"Nvdot":0,"Nrdot":-1.8,"Xu":1.98,"Yv":7.8,"Yr":0,"Nv":0,"Nr":3.87}
#asko={"L":63.83,"B":15,"D":3.7,"T":1.8,"m":1069,"xg":0,"Iz":0.51,"Xudot":-1.2,"Yvdot":-49.2,"Yrdot":0,"Nvdot":0,"Nrdot":-1.8,"Xu":1.98,"Yv":7.8,"Yr":0,"Nv":0,"Nr":3.87}
#===================================================
#Functions
#===================================================
def hydrostatics(file_name):
    desired_columns = ['Draft Amidships', 'Displ.', 'Wetted Area', 'Cp', 'Cb', 'Cwp', 'KG']
    # Dictionary to store tables
    tables = {}

    for table_name, info in table_info.items():
        # Extract table from the specified pages
        dfs = tabula.read_pdf(file_name, pages=info['pages'], multiple_tables=True)
        
    # Check if any DataFrame was returned, some PDFs can return multiple DataFrames
        if isinstance(dfs, list) and len(dfs) > 0:
            df = dfs[0]  # If multiple tables are returned, pick the first one (adjust if needed)
            
            # Check if the desired columns exist in the extracted DataFrame
            available_columns = [col for col in desired_columns if col in df.columns]

            # Filter the desired columns
            filtered_df = df[available_columns]
            
            # Add heel and trim as new columns
            filtered_df['Heel'] = info['heel']
            filtered_df['Trim'] = info['trim']
            
            # Save the filtered table to the dictionary
            tables[table_name] = filtered_df

            # Optionally print or check the table
            #print(f"{table_name} extracted and filtered with Heel and Trim added:")
            #print(filtered_df)
        else:
            print(f"No table found for {table_name}")

    # Example: Save all tables to a single Excel file with each table in a separate sheet
    with pd.ExcelWriter('hydrostatic_tables_with_heel_trim.xlsx') as writer:
        for table_name, table_df in tables.items():
            table_df.to_excel(writer, sheet_name=table_name, index=False)

    print("Extraction and saving of hydrostatics completed.")
